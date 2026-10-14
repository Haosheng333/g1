"""Readiness evaluation. Pure logic over an injected probe, so it tests offline.

The probe is the only thing that touches ROS. `Probe` documents the three
questions preflight asks; the node implements it with rospy and the tests
implement it with a dict.

Preflight runs to completion and reports EVERY blocker it finds. It never stops
at the first one: the point is to hand back the complete list so an operator can
fix everything in one pass instead of rediscovering blockers one restart at a
time.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from orchestrator import capabilities as caps


class Probe(object):
    """What preflight needs to know about the live system."""

    def has_service(self, name: str) -> bool:
        raise NotImplementedError

    def flag(self, topic: str):
        """Latest value of a latched Bool topic, or None if never received."""
        raise NotImplementedError

    def has_publisher(self, topic: str) -> bool:
        raise NotImplementedError

    def fresh(self, topic: str, max_age_s: float) -> str:
        """"" if the topic is arriving and recent, else the reason it is not."""
        raise NotImplementedError


@dataclass
class Check:
    capability: str
    owner: str
    ok: bool
    detail: str = ""
    #: True when the cause is an unresolved decision rather than a runtime state.
    unresolved: bool = False


@dataclass
class PreflightResult:
    checks: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)

    @property
    def blockers(self) -> list:
        return ["%s (%s): %s" % (c.capability, c.owner, c.detail) for c in self.checks if not c.ok]

    @property
    def unresolved_blockers(self) -> list:
        return [
            "%s (%s): %s" % (c.capability, c.owner, c.detail)
            for c in self.checks
            if not c.ok and c.unresolved
        ]

    def summary(self) -> str:
        if self.ok:
            return "preflight OK (%d capabilities)" % len(self.checks)
        return "preflight FAILED: " + " | ".join(self.blockers)


def check_capability(key: str, probe: Probe, side: str) -> Check:
    """Evaluate one capability. Collects every reason it is not ready."""
    cap = caps.capability(key)

    if cap.unresolved:
        return Check(key, cap.owner, False, cap.unresolved, unresolved=True)

    reasons = []
    for name in caps.resolve_services(cap, side):
        if not probe.has_service(name):
            reasons.append("service %s unavailable" % name)
    for topic in cap.ready_flags:
        value = probe.flag(topic)
        if value is None:
            reasons.append("%s not received (WAIT)" % topic)
        elif not value:
            reasons.append("%s is false" % topic)
    for topic in cap.topics:
        if not probe.has_publisher(topic):
            reasons.append("%s has no publisher" % topic)
    for topic, max_age in cap.fresh_topics:
        reason = probe.fresh(topic, max_age)
        if reason:
            reasons.append(reason)

    if reasons:
        return Check(key, cap.owner, False, "; ".join(reasons))
    return Check(key, cap.owner, True, "ready")


def run_preflight(route, probe: Probe, side: str) -> PreflightResult:
    """Evaluate every capability a route requires, in declaration order."""
    return PreflightResult(
        checks=[check_capability(key, probe, side) for key in route.requires]
    )
