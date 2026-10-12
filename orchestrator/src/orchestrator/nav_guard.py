"""The navigation terminal-state guard, as navigation itself specifies it.

This is the client side of navigation's contract and nothing more: it does no
planning, no costmap work and no velocity control. The logic mirrors
navigation/scripts/nav_client.py, which implements doc section 4.3, extracted
here with time injected so the guard is testable without a ROS master.

THE saw_active RULE. /navigation/status is latched and re-published every tick,
so a SUCCESS left over from the previous goal is still on the topic when a new
goal is accepted. A terminal state therefore only counts once an ACTIVE state
has been seen first. Without that rule the orchestrator would instantly
"succeed" on a stale message and march on to the next step while the robot had
not moved.
"""
from __future__ import annotations

#: In-progress states (navigation_node.py). PLANNING also covers RELOCALIZING,
#: which navigation reports as PLANNING because it is suspended, not finished.
ACTIVE = ("PLANNING", "FOLLOWING", "RECOVERING")
TERMINAL = ("SUCCESS", "FAILURE")

#: Guard verdicts.
SUCCESS = "SUCCESS"
FAILURE = "FAILURE"
TIMEOUT = "TIMEOUT"
PENDING = "PENDING"


class NavGuard(object):
    """Fold a stream of /navigation/status values into one verdict."""

    def __init__(self, timeout_s: float):
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = float(timeout_s)
        self._saw_active = False
        self._started_at = None
        self._verdict = PENDING
        self._reason = "not started"

    @property
    def saw_active(self) -> bool:
        return self._saw_active

    @property
    def verdict(self) -> str:
        return self._verdict

    @property
    def reason(self) -> str:
        return self._reason

    def start(self, now: float) -> None:
        self._saw_active = False
        self._started_at = float(now)
        self._verdict = PENDING
        self._reason = "waiting for an ACTIVE status"

    def update(self, now: float, status=None) -> str:
        """Feed the latest status (None = nothing on the topic yet)."""
        if self._verdict != PENDING:
            return self._verdict
        if self._started_at is None:
            raise RuntimeError("NavGuard.start() must be called before update()")

        if status in ACTIVE:
            self._saw_active = True
            self._reason = "navigation active: %s" % status
        elif status in TERMINAL:
            if self._saw_active:
                self._verdict = SUCCESS if status == SUCCESS else FAILURE
                self._reason = "navigation reported %s after becoming active" % status
                return self._verdict
            # Stale terminal state from a previous goal; keep waiting.
            self._reason = (
                "ignoring %s: no ACTIVE status seen yet, so this is stale (saw_active guard)"
                % status
            )
        elif status is None:
            self._reason = "no status on /navigation/status yet"
        else:
            self._reason = "unrecognised status %r" % (status,)

        if (float(now) - self._started_at) > self.timeout_s:
            self._verdict = TIMEOUT
            self._reason = "no terminal status within %.1fs (saw_active=%s)" % (
                self.timeout_s,
                self._saw_active,
            )
        return self._verdict
