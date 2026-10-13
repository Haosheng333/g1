"""Configurable bridge from a tactile ROS message to one scalar.

DISABLED BY DEFAULT, and that is the point. As of this commit the repository
publishes no tactile topic at all: hand_control talks to the BrainCo Revo2 over
serial and uses finger-position feedback internally (`expect_contact` in
config/finger_presets.yaml); it exposes only three std_srvs/Trigger services
and no sensor topic.

So the real message type, its field layout and its units are unknown. Rather
than invent them, this adapter is driven entirely by configuration:

    topic          where to subscribe
    message_type   "pkg/Msg", imported at runtime, never hard-coded here
    value_field    dotted path into the message, e.g. "data" or "wrench.force.z"
    reduction      how to collapse a sequence field to one scalar

With `enabled: false` (the default) nothing is subscribed and
`describe_blocker()` explains what the hand owner has to confirm. The monitor
then refuses to arm, so no handover can be triggered off a guessed schema.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

#: Supported ways to collapse a sequence-valued field into one scalar.
REDUCTIONS = ("mean", "max", "min", "sum", "first", "norm")


@dataclass
class AdapterConfig:
    enabled: bool = False
    topic: str = ""
    message_type: str = ""
    value_field: str = ""
    reduction: str = "mean"

    def validate(self) -> list:
        """Reasons this adapter cannot be used (empty = usable)."""
        if not self.enabled:
            return [
                "tactile adapter is disabled (handover/config/auto_release.yaml: "
                "tactile.enabled=false). No tactile topic exists in this workspace yet: "
                "hand_control reads the Revo2 over serial and publishes no sensor topic. "
                "The hand owner must confirm the topic, message type, field and units "
                "before this can be enabled."
            ]
        problems = []
        if not self.topic:
            problems.append("tactile.topic is empty")
        if not self.message_type:
            problems.append("tactile.message_type is empty (expected \"pkg/Msg\")")
        elif self.message_type.count("/") != 1 or not all(self.message_type.split("/")):
            problems.append(
                "tactile.message_type must look like \"pkg/Msg\", got %r" % (self.message_type,)
            )
        if not self.value_field:
            problems.append("tactile.value_field is empty (dotted path into the message)")
        if self.reduction not in REDUCTIONS:
            problems.append(
                "tactile.reduction must be one of %s, got %r"
                % (", ".join(REDUCTIONS), self.reduction)
            )
        return problems

    def describe_blocker(self) -> str:
        problems = self.validate()
        return "" if not problems else "tactile adapter unusable: " + "; ".join(problems)


def resolve_message_class(message_type: str):
    """Import a message class from its "pkg/Msg" name at runtime.

    Kept behind the enabled flag so a disabled adapter never imports anything.
    """
    if message_type.count("/") != 1:
        raise ValueError("message_type must look like \"pkg/Msg\", got %r" % (message_type,))
    package, name = message_type.split("/")
    module = __import__("%s.msg" % package, fromlist=[name])
    try:
        return getattr(module, name)
    except AttributeError:
        raise ValueError("%s has no message %r" % (package, name))


def get_field(message, dotted: str):
    """Follow a dotted attribute path into a message. Raises KeyError if absent."""
    if not dotted:
        raise KeyError("empty field path")
    current = message
    for part in dotted.split("."):
        if not hasattr(current, part):
            raise KeyError("field %r not found (at %r)" % (dotted, part))
        current = getattr(current, part)
    return current


def reduce_value(value, reduction: str) -> float:
    """Collapse a scalar or sequence into one finite float."""
    if reduction not in REDUCTIONS:
        raise ValueError("unknown reduction %r" % (reduction,))

    if isinstance(value, (bool,)):
        raise TypeError("tactile field is a bool, which is not a measurement")
    if isinstance(value, (int, float)):
        values = [float(value)]
    else:
        try:
            values = [float(v) for v in value]
        except (TypeError, ValueError):
            raise TypeError("tactile field is neither a number nor a sequence of numbers")
    if not values:
        raise ValueError("tactile field is an empty sequence")
    if not all(math.isfinite(v) for v in values):
        raise ValueError("tactile field contains a non-finite value")

    if reduction == "mean":
        return sum(values) / float(len(values))
    if reduction == "max":
        return max(values)
    if reduction == "min":
        return min(values)
    if reduction == "sum":
        return float(sum(values))
    if reduction == "first":
        return values[0]
    return math.sqrt(sum(v * v for v in values))   # norm


def extract_scalar(message, config: AdapterConfig) -> float:
    """message -> scalar, per `config`. Raises on anything unexpected."""
    return reduce_value(get_field(message, config.value_field), config.reduction)


# =====================================================================================
# Multi-channel adapter
# =====================================================================================
#
# The single-value AdapterConfig above remains for the scalar case. What follows
# maps ONE tactile message onto the multi-channel frame the tactile detector
# consumes.
#
# EVERY mapping comes from configuration. Nothing here knows what fields a real
# BrainCo Revo2 message has, because that schema is unconfirmed: the vendor
# documents four Revo2 variants (Basic has NO sensor at all; Touch, TouchForce3D
# and TouchArrayPressure differ) and the authoritative field list lives in the
# SDK's `main_mod.pyi`, which is not available here. See handover/README.md.


@dataclass
class ChannelMapping:
    """One declared channel and where to read it from in the message."""

    name: str
    finger: str
    kind: str                 # force_like | angular | opaque (tactile_frame kinds)
    field: str                # dotted/indexed path, e.g. "data[0]" or "fingers.index.fz"
    unit: str = "unknown"
    period: float = 0.0       # angular only; 0 = use the ChannelSpec default

    def validate(self) -> list:
        from handover import tactile_frame as _tf

        problems = []
        if not self.name:
            problems.append("a channel mapping has an empty name")
        if self.kind not in _tf.KINDS:
            problems.append(
                "channel %r kind %r must be one of %s"
                % (self.name, self.kind, ", ".join(_tf.KINDS)))
        if not self.field:
            problems.append("channel %r has an empty field path" % (self.name,))
        if self.kind == _tf.ANGULAR and self.period and self.period <= 0:
            problems.append("channel %r angular period must be positive" % (self.name,))
        return problems

    def to_spec(self):
        from handover import tactile_frame as _tf

        kw = {}
        if self.period:
            kw["period"] = float(self.period)
        return _tf.ChannelSpec(
            name=self.name, finger=self.finger, kind=self.kind, unit=self.unit, **kw)


@dataclass
class TactileAdapterConfig:
    """How to turn one tactile message into a TactileFrame. Disabled by default."""

    enabled: bool = False
    topic: str = ""
    message_type: str = ""
    #: Optional dotted path to a message timestamp in seconds. Empty = use the
    #: receipt time, which is less accurate and is reported as such.
    stamp_field: str = ""
    #: Optional dotted path to a per-frame validity flag published by the driver.
    valid_field: str = ""
    #: Expected publish rate, used only for diagnostics and staleness advice.
    expected_rate_hz: float = 0.0
    mappings: tuple = ()

    def validate(self) -> list:
        if not self.enabled:
            return [
                "tactile adapter is disabled (handover/config/auto_release.yaml: "
                "tactile.enabled=false). The BrainCo Revo2 tactile schema is NOT "
                "confirmed: the vendor ships four variants (Basic has no sensor) and the "
                "authoritative field list is in the SDK's main_mod.pyi, which is not "
                "available in this workspace. The hand owner must supply the hand "
                "variant, that stub, and a sample frame before this can be enabled."
            ]
        problems = []
        if not self.topic:
            problems.append("tactile.topic is empty")
        if not self.message_type:
            problems.append("tactile.message_type is empty (expected \"pkg/Msg\")")
        elif self.message_type.count("/") != 1 or not all(self.message_type.split("/")):
            problems.append(
                "tactile.message_type must look like \"pkg/Msg\", got %r" % (self.message_type,))
        if not self.mappings:
            problems.append("no tactile channels are mapped")
        seen = set()
        for m in self.mappings:
            problems.extend(m.validate())
            if m.name in seen:
                problems.append("channel %r is mapped more than once" % (m.name,))
            seen.add(m.name)
        return problems

    def describe_blocker(self) -> str:
        problems = self.validate()
        return "" if not problems else "tactile adapter unusable: " + "; ".join(problems)

    def specs(self) -> dict:
        return {m.name: m.to_spec() for m in self.mappings}


def get_indexed_field(message, path: str):
    """Follow a dotted path that may contain list indices: `a.b[2].c`."""
    import re as _re

    current = message
    for part in path.split("."):
        m = _re.match(r"^([A-Za-z_][A-Za-z0-9_]*)((?:\[\d+\])*)$", part)
        if not m:
            raise KeyError("malformed field path segment %r in %r" % (part, path))
        name, idx = m.group(1), m.group(2)
        if not hasattr(current, name):
            if isinstance(current, dict) and name in current:
                current = current[name]
            else:
                raise KeyError("field %r not found (at %r)" % (path, name))
        else:
            current = getattr(current, name)
        for n in _re.findall(r"\[(\d+)\]", idx):
            try:
                current = current[int(n)]
            except (IndexError, KeyError, TypeError):
                raise KeyError("index [%s] out of range in %r" % (n, path))
    return current


def extract_frame(message, config: TactileAdapterConfig, receipt_time: float):
    """message -> TactileFrame. Raises KeyError/TypeError/ValueError on bad input.

    The caller treats any exception as a fault; the detector then refuses to
    count the frame as evidence and requires a fresh baseline.
    """
    from handover import tactile_frame as _tf

    stamp = receipt_time
    if config.stamp_field:
        stamp = float(reduce_value(get_indexed_field(message, config.stamp_field), "first"))

    source_valid = True
    if config.valid_field:
        source_valid = bool(get_indexed_field(message, config.valid_field))

    values = {}
    for m in config.mappings:
        values[m.name] = float(
            reduce_value(get_indexed_field(message, m.field), "first"))
    return _tf.TactileFrame(stamp=stamp, values=values, source_valid=source_valid)
