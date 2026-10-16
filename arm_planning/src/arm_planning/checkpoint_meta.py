"""Read a trained IK checkpoint's embedded kinematics config, without torch.

WHY THIS EXISTS
Each `models/g1_<arm>_hybrid_best.pt` embeds, under its `config` key, a copy of
the whole `config/arm_kinematics.yaml` it was trained against -- including
`grasp_xyz`. The network learns a pose -> joint mapping in which "pose" means
the tool point as it was defined AT TRAINING TIME. Editing `grasp_xyz` in the
YAML therefore silently redefines the target frame the checkpoint was fitted to,
and nothing in the pipeline noticed: no loader compared the two.

That is not a hypothetical. The 2026-10-14 tool-point correction (y 0.006 ->
0.06) moved the tool point by 54 mm, against checkpoints whose own recorded
validation error is ~4.2-4.4 mm. A 54 mm redefinition is an order of magnitude
larger than the accuracy the checkpoint was accepted at, so the two must not be
assumed compatible.

HOW IT READS THE FILE
A `.pt` is a zip whose `data.pkl` is an ordinary pickle; only the tensor
rebuilds reference torch. Stubbing those out lets the plain-Python parts -- the
embedded config, `arm`, `epoch`, the validation error -- unpickle with no torch
installed, which matters because torch is absent from this workspace and from
the ROS container. No tensor is materialised and no weight is read.

This module performs no I/O beyond opening the checkpoint, imports no ROS and
loads no model.
"""
from __future__ import annotations

import io
import math
import pickle
import zipfile
from dataclasses import dataclass, field

#: Checkpoint basename for an arm, relative to the models directory.
CHECKPOINT_NAME = "g1_%s_hybrid_best.pt"

#: Keys compared between the YAML and the embedded copy. Each is a per-arm
#: quantity that changes what a trained pose MEANS, so a difference invalidates
#: the mapping rather than merely perturbing it.
TOOL_POINT_KEYS = ("grasp_xyz", "grasp_rpy")

#: Tolerance for declaring two tool points equal, in metres/radians. Well below
#: any plausible real difference and well above float round-trip noise.
EQUAL_TOL = 1e-9


class CheckpointReadError(RuntimeError):
    """The checkpoint could not be opened or carried no readable config."""


class _Stub(object):
    """Stands in for any torch object encountered while unpickling."""

    def __init__(self, *_a, **_k):
        pass

    def __repr__(self):
        return "<stub>"


def _stub_factory(*_a, **_k):
    return _Stub()


class _StubUnpickler(pickle.Unpickler):
    """Unpickles the plain-Python parts, stubbing anything torch-shaped."""

    def find_class(self, module, name):
        if module.startswith("torch") or name in ("_rebuild_tensor_v2", "_load_from_bytes"):
            return _stub_factory
        return super().find_class(module, name)

    def persistent_load(self, pid):
        return _Stub()


def read_checkpoint_meta(path) -> dict:
    """Return the checkpoint's non-tensor top level as a dict.

    Raises CheckpointReadError for a missing/corrupt file or one whose payload
    is not the expected mapping.
    """
    try:
        archive = zipfile.ZipFile(str(path))
    except (OSError, zipfile.BadZipFile) as exc:
        raise CheckpointReadError("cannot open %s: %s" % (path, exc))

    with archive:
        pickles = [n for n in archive.namelist() if n.endswith("/data.pkl")]
        if not pickles:
            raise CheckpointReadError("%s contains no data.pkl" % (path,))
        try:
            raw = archive.read(pickles[0])
            payload = _StubUnpickler(io.BytesIO(raw)).load()
        except Exception as exc:                      # malformed pickle, odd protocol
            raise CheckpointReadError("cannot read %s: %s: %s" % (path, type(exc).__name__, exc))

    if not isinstance(payload, dict):
        raise CheckpointReadError(
            "%s unpickled to %s, expected a mapping" % (path, type(payload).__name__))
    return payload


def embedded_config(path) -> dict:
    """The `config` mapping a checkpoint was trained with."""
    meta = read_checkpoint_meta(path)
    config = meta.get("config")
    if not isinstance(config, dict):
        raise CheckpointReadError(
            "%s has no embedded 'config' mapping, so the tool point it was trained "
            "with cannot be established" % (path,))
    return config


def _vectors_differ(a, b) -> bool:
    try:
        if len(a) != len(b):
            return True
        return any(not math.isclose(float(x), float(y), abs_tol=EQUAL_TOL)
                   for x, y in zip(a, b))
    except (TypeError, ValueError):
        return True


@dataclass
class ToolPointComparison:
    """Per-arm result of comparing a live config against a checkpoint's copy."""

    arm: str
    checkpoint: str = ""
    #: key -> (live_value, checkpoint_value) for every key that differs.
    differences: dict = field(default_factory=dict)
    #: Euclidean distance between the two grasp_xyz values, metres.
    translation_delta_m: float = 0.0
    #: Validation error the checkpoint recorded, metres, when it carried one.
    recorded_validation_m: float = 0.0
    #: Populated instead of the above when the checkpoint could not be read.
    error: str = ""

    @property
    def consistent(self) -> bool:
        return not self.error and not self.differences

    def describe(self) -> str:
        if self.error:
            return "%s: %s" % (self.arm, self.error)
        if not self.differences:
            return "%s: tool point matches the checkpoint" % (self.arm,)
        parts = ["%s live=%r checkpoint=%r" % (k, live, ck)
                 for k, (live, ck) in sorted(self.differences.items())]
        detail = "; ".join(parts)
        if self.translation_delta_m:
            detail += "; tool point moved %.4f m" % (self.translation_delta_m,)
            if self.recorded_validation_m:
                detail += " vs the checkpoint's own %.4f m validation error (%.0fx)" % (
                    self.recorded_validation_m,
                    self.translation_delta_m / self.recorded_validation_m)
        return "%s: STALE -- %s" % (self.arm, detail)


def compare_tool_point(live_config: dict, checkpoint_path, arm: str) -> ToolPointComparison:
    """Compare one arm's tool point in `live_config` against the checkpoint's copy."""
    result = ToolPointComparison(arm=arm, checkpoint=str(checkpoint_path))
    try:
        meta = read_checkpoint_meta(checkpoint_path)
        embedded = meta.get("config")
        if not isinstance(embedded, dict):
            raise CheckpointReadError("no embedded 'config' mapping")
    except CheckpointReadError as exc:
        result.error = str(exc)
        return result

    recorded = meta.get("best_validation_position_mean_m")
    try:
        result.recorded_validation_m = float(recorded)
    except (TypeError, ValueError):
        result.recorded_validation_m = 0.0

    live_arm = live_config.get(arm) or {}
    ck_arm = embedded.get(arm) or {}
    if not isinstance(live_arm, dict) or not isinstance(ck_arm, dict):
        result.error = "arm %r is missing from the live config or the checkpoint's copy" % (arm,)
        return result

    for key in TOOL_POINT_KEYS:
        live_value, ck_value = live_arm.get(key), ck_arm.get(key)
        if _vectors_differ(live_value, ck_value):
            result.differences[key] = (live_value, ck_value)

    if "grasp_xyz" in result.differences:
        live_xyz, ck_xyz = result.differences["grasp_xyz"]
        try:
            result.translation_delta_m = math.sqrt(
                sum((float(a) - float(b)) ** 2 for a, b in zip(live_xyz, ck_xyz)))
        except (TypeError, ValueError):
            result.translation_delta_m = 0.0

    return result


def compare_all(live_config: dict, models_dir, arms=("left", "right")) -> list:
    """One ToolPointComparison per arm, in the order given."""
    from pathlib import Path

    models = Path(str(models_dir))
    return [compare_tool_point(live_config, models / (CHECKPOINT_NAME % a), a) for a in arms]


def stale_arms(comparisons) -> list:
    """Arms whose checkpoint no longer matches the live tool point."""
    return [c.arm for c in comparisons if c.differences]
