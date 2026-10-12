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
