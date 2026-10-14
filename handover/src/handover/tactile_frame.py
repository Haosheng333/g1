"""Tactile frame and channel model. No ROS, no vendor SDK, no hardware assumption.

WHY A CHANNEL *KIND* EXISTS
The framework says a frame may carry normal force, tangential force, direction
and proximity, in raw sensor counts. Those are not interchangeable:

  * a force-like channel is a scalar you may subtract and threshold;
  * a direction channel may be an angle, where 359 deg and 1 deg are 2 deg
    apart, so plain subtraction is wrong;
  * a proximity channel may be in unrelated units, inverted, or non-linear.

So every channel is declared with a `kind`, and the detector refuses to
threshold anything it has not been told how to compare. Nothing here decides
which channels a real hand actually has: that comes from configuration, once
the SDK schema is confirmed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# --- channel kinds -------------------------------------------------------------------

#: A scalar measurement where `new - baseline` is meaningful and may be
#: thresholded. Normal force and tangential force are expected to be these.
FORCE_LIKE = "force_like"

#: An angle. Differences must be wrapped into [-pi, pi] (or the configured
#: period). NOT used for magnitude triggering unless explicitly enabled,
#: because a direction change alone is not evidence of a pull.
ANGULAR = "angular"

#: Anything whose units, sign convention or linearity is not established:
#: proximity, raw counts, capacitance. Recorded and published, NEVER compared
#: against a threshold. Promote it to FORCE_LIKE only after calibration shows
#: the subtraction is meaningful.
OPAQUE = "opaque"

KINDS = (FORCE_LIKE, ANGULAR, OPAQUE)

#: Kinds a release decision may be based on.
TRIGGERABLE_KINDS = (FORCE_LIKE,)


@dataclass(frozen=True)
class ChannelSpec:
    """One scalar channel of one finger.

    `name` is the key used in a TactileFrame and in the threshold config.
    `unit` is free text carried through to recordings and diagnostics; it is
    never parsed, because the real units are unknown until the SDK is confirmed.
    """

    name: str
    finger: str
    kind: str
    unit: str = "unknown"
    #: Period for ANGULAR channels (radians by default; set 360.0 for degrees).
    period: float = 2.0 * math.pi

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(
                "channel %r has kind %r, expected one of %s"
                % (self.name, self.kind, ", ".join(KINDS))
            )
        if self.kind == ANGULAR and not (self.period > 0 and math.isfinite(self.period)):
            raise ValueError("angular channel %r needs a finite positive period" % (self.name,))

    @property
    def triggerable(self) -> bool:
        return self.kind in TRIGGERABLE_KINDS


def difference(spec: ChannelSpec, new: float, baseline: float) -> float:
    """Signed difference appropriate to the channel's kind.

    Raises for OPAQUE: there is no defensible difference until someone
    establishes what the numbers mean.
    """
    if spec.kind == FORCE_LIKE:
        return float(new) - float(baseline)
    if spec.kind == ANGULAR:
        half = spec.period / 2.0
        d = (float(new) - float(baseline)) % spec.period
        return d - spec.period if d > half else d
    raise ValueError(
        "channel %r is kind %r: no meaningful difference is defined. Declare it "
        "force_like only after calibration shows subtraction is valid." % (spec.name, spec.kind)
    )


# --- frames --------------------------------------------------------------------------


@dataclass
class TactileFrame:
    """One multi-channel reading.

    `stamp` is seconds (the publisher's clock). `values` maps channel name ->
    number. `source_valid` lets a publisher mark a frame the hardware itself
    flagged as bad, independently of whether the numbers parse.
    """

    stamp: float
    values: dict = field(default_factory=dict)
    source_valid: bool = True


@dataclass
class FrameCheck:
    """Result of validating a frame against the declared channels."""

    ok: bool
    reason: str = ""
    #: Channel name -> float, only the channels that were required and valid.
    clean: dict = field(default_factory=dict)


def validate_frame(frame, specs, required_names) -> FrameCheck:
    """Reject anything that must never count as evidence of a pull.

    A frame is usable only if it exists, is marked valid by its source, has a
    finite stamp, and carries a finite number for EVERY required channel. A
    partially present frame is rejected rather than silently masked, because a
    missing finger is a fault, not a zero reading.
    """
    if frame is None:
        return FrameCheck(False, "no tactile frame available")
    if not getattr(frame, "source_valid", True):
        return FrameCheck(False, "frame marked invalid by its source")

    stamp = getattr(frame, "stamp", None)
    try:
        stamp = float(stamp)
    except (TypeError, ValueError):
        return FrameCheck(False, "frame stamp is not numeric: %r" % (stamp,))
    if not math.isfinite(stamp):
        return FrameCheck(False, "frame stamp is not finite: %r" % (stamp,))

    values = getattr(frame, "values", None)
    if not isinstance(values, dict):
        return FrameCheck(False, "frame values must be a mapping, got %r" % (type(values).__name__,))

    clean = {}
    for name in required_names:
        if name not in specs:
            return FrameCheck(False, "channel %r is not declared" % (name,))
        if name not in values:
            return FrameCheck(False, "frame is missing required channel %r" % (name,))
        raw = values[name]
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            return FrameCheck(False, "channel %r is not a number: %r" % (name, raw))
        v = float(raw)
        if not math.isfinite(v):
            return FrameCheck(False, "channel %r is not finite: %r" % (name, raw))
        clean[name] = v
    return FrameCheck(True, "ok", clean)
