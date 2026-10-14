"""ROS-free request validation, asset resolution and path flattening.

Everything here is pure python + stdlib so the offline tests run without ROS,
torch, scipy, trimesh or fcl installed. The service field names mirror the
Module 2 specification section 11.4 ("Reconstruct flattened paths").
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

# --- conventions fixed by the specification ------------------------------------------

#: Arm joint order, doc section 2.3. Every path this package produces or
#: consumes is in this order, and arm_control writes the same five indices
#: (arm_controller.py JointIndex.*ShoulderPitch/Roll/Yaw/Elbow/WristRoll).
JOINT_ORDER = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll")

#: Degrees of freedom per arm (doc 2.3).
DOF = 5

ARMS = ("left", "right")

#: Grasp segment order, doc section 6.1. The orchestrator opens the gripper
#: after `to_pregrasp` and closes it after `approach`.
SEGMENT_ORDER = ("to_pregrasp", "approach", "lift", "retract")


def expected_joint_names(arm: str) -> list[str]:
    """URDF joint names for `arm`, in JOINT_ORDER.

    Matches `arm_kinematics.yaml` keys `left.joint_names` / `right.joint_names`.
    """
    if arm not in ARMS:
        raise ValueError("arm must be 'left' or 'right', got %r" % (arm,))
    return ["%s_%s_joint" % (arm, j) for j in JOINT_ORDER]


# --- request validation --------------------------------------------------------------
#
# Mirrors the rejection conditions of doc section 12.2 that can be checked
# before the planner is involved at all. Anything deeper (workspace, IK,
# collision) is the planner's own decision and its message is passed through.


def validate_arm(arm) -> str | None:
    if arm not in ARMS:
        return "arm must be 'left' or 'right', got %r" % (arm,)
    return None


def validate_vector(values, length: int, name: str) -> str | None:
    """Require exactly `length` finite real numbers."""
    try:
        seq = list(values)
    except TypeError:
        return "%s must be a sequence of %d numbers" % (name, length)
    if len(seq) != length:
        return "%s must contain exactly %d values, got %d" % (name, length, len(seq))
    for i, v in enumerate(seq):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return "%s[%d] must be a number, got %r" % (name, i, v)
        if not math.isfinite(float(v)):
            return "%s[%d] must be finite, got %r" % (name, i, v)
    return None


def validate_plan_to_pose_request(arm, current_joints, target_pose) -> str | None:
    """None when the request is well-formed, else the reason to reject it."""
    return (
        validate_arm(arm)
        or validate_vector(current_joints, DOF, "current_joints")
        or validate_vector(target_pose, DOF, "target_pose")
    )


def validate_plan_grasp_request(arm, current_joints, object_xyz) -> str | None:
    """None when the request is well-formed, else the reason to reject it."""
    return (
        validate_arm(arm)
        or validate_vector(current_joints, DOF, "current_joints")
        or validate_vector(object_xyz, 3, "object_xyz")
    )


# --- path flattening (doc 11.4) ------------------------------------------------------


def flatten_path(joint_path) -> tuple[list[float], int]:
    """N x DOF joint path -> (row-major flat list, N).

    Raises ValueError on a ragged path, because a partially valid path must
    never reach a controller (doc 12.1: an empty or malformed path is never a
    recoverable command).
    """
    rows = list(joint_path or [])
    flat: list[float] = []
    for i, row in enumerate(rows):
        values = list(row)
        if len(values) != DOF:
            raise ValueError(
                "waypoint %d has %d values, expected %d (joint order %s)"
                % (i, len(values), DOF, ", ".join(JOINT_ORDER))
            )
        for j, v in enumerate(values):
            f = float(v)
            if not math.isfinite(f):
                raise ValueError("waypoint %d joint %d is not finite: %r" % (i, j, v))
            flat.append(f)
    return flat, len(rows)


def flatten_segments(segments, order=SEGMENT_ORDER):
    """{name: {'joint_path': N x DOF}} -> (names, counts, concatenated flat list).

    `order` fixes the output order, so the orchestrator's gripper steps always
    land between the right segments. A missing segment is an error, not a gap.
    """
    names: list[str] = []
    counts: list[int] = []
    flat: list[float] = []
    for name in order:
        if name not in (segments or {}):
            raise ValueError(
                "missing grasp segment %r (expected all of: %s)" % (name, ", ".join(order))
            )
        segment = segments[name]
        path = segment["joint_path"] if isinstance(segment, dict) else segment.joint_path
        seg_flat, count = flatten_path(path)
        if count == 0:
            raise ValueError("grasp segment %r has no waypoints" % (name,))
        names.append(name)
        counts.append(count)
        flat.extend(seg_flat)
    return names, counts, flat


def unflatten_path(flat, waypoint_count: int) -> list[list[float]]:
    """Inverse of flatten_path; used by the tests and by any python consumer."""
    values = list(flat)
    if waypoint_count < 0:
        raise ValueError("waypoint_count must not be negative")
    if len(values) != waypoint_count * DOF:
        raise ValueError(
            "flat path has %d values, expected %d (%d waypoints x %d)"
            % (len(values), waypoint_count * DOF, waypoint_count, DOF)
        )
    return [values[i * DOF:(i + 1) * DOF] for i in range(waypoint_count)]


def nan_if_none(value) -> float:
    """A ROS float64 cannot carry None, so an unmeasured error becomes NaN.

    The planner leaves position/rx/rz error as None when verification never
    ran (doc 5.2 fields are optional on failure).
    """
    return float("nan") if value is None else float(value)


# --- runtime asset resolution --------------------------------------------------------
#
# Every path is resolved against an explicit project root (the catkin package
# share directory, found with rospkg in the node) and NEVER against the current
# working directory. The upstream planner defaults are cwd-relative
# (module2_planner.py: "config/arm_kinematics.yaml", "models_hybrid_1m/...").

#: Checkpoint directories to search, in order. The specification's file
#: inventory (doc 8.1) names `models_hybrid_1m/`; this repository ships the
#: checkpoints in `models/`. Both are accepted and the resolved one is
#: reported, so neither location is silently assumed.
CHECKPOINT_DIRS = ("models", "models_hybrid_1m")

#: Mesh files that arm_planning/scripts/g1_collision_probe.py loads when it
#: builds its FCL objects. Kept in sync with that file by
#: test_asset_resolution.test_required_meshes_match_collision_probe_source.
STATIC_BODY_MESHES = (
    "torso_link.STL",
    "head_link.STL",
    "pelvis.STL",
    "waist_yaw_link.STL",
    "waist_support_link.STL",
)
PER_ARM_MESH_TEMPLATES = (
    "{arm}_shoulder_yaw_link.STL",
    "{arm}_elbow_link.STL",
    "{arm}_wrist_roll_rubber_hand.STL",
)


def required_mesh_files(arms=ARMS) -> list[str]:
    """Mesh filenames the collision probe needs for `arms`, de-duplicated."""
    names = list(STATIC_BODY_MESHES)
    for arm in arms:
        names.extend(t.format(arm=arm) for t in PER_ARM_MESH_TEMPLATES)
    seen: list[str] = []
    for n in names:
        if n not in seen:
            seen.append(n)
    return seen


def scan_collision_probe_meshes(probe_source: str) -> list[str]:
    """Extract the .STL filenames referenced by g1_collision_probe.py source.

    Used by the tests to prove `required_mesh_files()` has not drifted from the
    probe. `{prefix}_x.STL` f-strings are expanded for both arms.
    """
    found: list[str] = []
    for raw in re.findall(r"""['"]([A-Za-z0-9_{}]+\.STL)['"]""", probe_source):
        if "{prefix}" in raw:
            for arm in ARMS:
                found.append(raw.replace("{prefix}", arm))
        else:
            found.append(raw)
    out: list[str] = []
    for n in found:
        if n not in out:
            out.append(n)
    return out


@dataclass
class AssetPaths:
    """Resolved Module 2 runtime assets (doc 8.1) plus what is missing."""

    project_root: Path
    config_path: Path
    checkpoints: dict = field(default_factory=dict)
    masks: dict = field(default_factory=dict)
    meshes_dir: Path | None = None
    missing: list = field(default_factory=list)
    checkpoint_dir_used: str | None = None

    @property
    def ready(self) -> bool:
        return not self.missing

    def blocker_message(self) -> str:
        if self.ready:
            return ""
        return "missing Module 2 assets under %s: %s" % (
            self.project_root,
            "; ".join(self.missing),
        )


def resolve_assets(project_root, checkpoint_dirs=CHECKPOINT_DIRS, arms=ARMS) -> AssetPaths:
    """Resolve config, checkpoints, masks and meshes under `project_root`.

    Pure function: takes the root as an argument and touches no global state,
    so the tests can point it at a temporary directory. Never raises for a
    missing asset -- the caller reports `missing` as a NOT_READY blocker.
    """
    root = Path(project_root)
    config_path = root / "config" / "arm_kinematics.yaml"
    assets = AssetPaths(project_root=root, config_path=config_path)

    if not config_path.is_file():
        assets.missing.append("config/arm_kinematics.yaml")

    # Checkpoints: first directory that holds BOTH arms wins, so a half-populated
    # directory cannot shadow a complete one.
    for candidate in checkpoint_dirs:
        paths = {arm: root / candidate / ("g1_%s_hybrid_best.pt" % arm) for arm in arms}
        if all(p.is_file() for p in paths.values()):
            assets.checkpoints = paths
            assets.checkpoint_dir_used = candidate
            break
    if not assets.checkpoints:
        assets.missing.append(
            "IK checkpoints g1_{%s}_hybrid_best.pt in any of: %s"
            % (",".join(arms), ", ".join(checkpoint_dirs))
        )

    for arm in arms:
        mask = root / "collision_masks" / ("%s_static_collision_mask.npy" % arm)
        assets.masks[arm] = mask
        if not mask.is_file():
            assets.missing.append("collision_masks/%s_static_collision_mask.npy" % arm)

    meshes_dir = root / "meshes"
    assets.meshes_dir = meshes_dir
    if not meshes_dir.is_dir():
        assets.missing.append("meshes/ directory")
    else:
        absent = [m for m in required_mesh_files(arms) if not (meshes_dir / m).is_file()]
        if absent:
            assets.missing.append(
                "collision meshes %s (download from the URL in meshes/README.md)"
                % ", ".join(absent)
            )
    return assets
