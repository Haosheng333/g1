"""Who is holding what, and whether a release request is legitimate.

WHY THIS IS NOT THE EXECUTION LOCK
`ExecutionLock` is a non-reentrant, non-blocking lock held for the WHOLE of a
route. A handover task holds it while it waits for a human to take the object.
If the tactile monitor's release request tried to run a route, it would call
`ExecutionLock.hold()`, find the lock held by the very task it is trying to
serve, and be refused outright -- so the legitimate release could never get
through. (It would not deadlock, because the lock is non-blocking, but refusing
every real release is just as broken.)

So this module does the opposite: the service callback only VALIDATES the
request and RECORDS it against the active task, using this object's own small
mutex. The task that already owns the execution lock then consumes the recorded
event and performs the release under its existing ownership. There is exactly
one motion path, and the callback is not on it.

This module performs no I/O, holds no ROS handle and moves nothing.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field

# --- task phases ---------------------------------------------------------------------

#: Nothing held, no handover in progress.
IDLE = "IDLE"
#: Holding an object; the arm is moving to the handover pose. The detector must
#: NOT be armed in this phase.
MOVING_TO_POSE = "MOVING_TO_POSE"
#: At the pose, settled, detector armed, waiting for a human pull. This is the
#: ONLY phase in which a release request can be accepted.
AWAITING_PULL = "AWAITING_PULL"
#: A validated pull was consumed; the owning task is releasing.
RELEASING = "RELEASING"

PHASES = (IDLE, MOVING_TO_POSE, AWAITING_PULL, RELEASING)


@dataclass
class Validation:
    """Outcome of validating a release request. Never performs the release."""

    ok: bool
    reason: str = ""
    #: Monotonic id of the recorded event, when one was recorded.
    sequence: int = 0


@dataclass
class PendingRelease:
    """A validated pull, waiting for the owning task to act on it."""

    sequence: int
    session: int
    side: str
    requested_at: float
    source: str = ""


@dataclass
class Snapshot:
    """Read-only view, for diagnostics and tests."""

    phase: str = IDLE
    side: str = ""
    label: str = ""
    session: int = 0
    holding: bool = False
    release_authorized: bool = False
    pending: bool = False
    consumed_this_session: bool = False
    awaiting_since: float = 0.0


class HandoverState(object):
    """Task-owned handover record. Thread-safe via its OWN lock."""

    def __init__(self, max_awaiting_sec: float = 120.0):
        self._lock = threading.Lock()
        self._max_awaiting_sec = float(max_awaiting_sec)
        self._phase = IDLE
        self._side = ""
        self._label = ""
        self._holding = False
        self._release_authorized = False
        self._session = 0
        self._awaiting_since = 0.0
        self._pending = None
        self._consumed_session = -1
        self._sequence = 0

    # ------------------------------------------------------------------ task side
    def begin(self, side: str, label: str, release_authorized: bool) -> None:
        """Record that a task now holds `label` in `side` and is moving."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right', got %r" % (side,))
        with self._lock:
            self._phase = MOVING_TO_POSE
            self._side = side
            self._label = label
            self._holding = True
            self._release_authorized = bool(release_authorized)
            self._session += 1
            self._pending = None
            self._awaiting_since = 0.0

    def mark_awaiting_pull(self, now: float) -> None:
        """The arm has settled at the handover pose and the detector is armed."""
        with self._lock:
            if not self._holding:
                raise RuntimeError("cannot await a pull without a held object")
            self._phase = AWAITING_PULL
            self._awaiting_since = float(now)
            self._pending = None

    def mark_releasing(self) -> None:
        with self._lock:
            self._phase = RELEASING

    def mark_released(self) -> None:
        """The hand opened and the object is no longer held."""
        with self._lock:
            self._phase = IDLE
            self._holding = False
            self._label = ""
            self._pending = None

    def abandon(self, reason: str = "") -> None:
        """Give up the handover without releasing (timeout, fault, disarm)."""
        with self._lock:
            self._phase = MOVING_TO_POSE if self._holding else IDLE
            self._pending = None
            self._awaiting_since = 0.0

    def consume_pending(self, now: float):
        """Take the validated event, if any. Called by the task that owns the lock."""
        with self._lock:
            event = self._pending
            if event is None:
                return None
            self._pending = None
            self._consumed_session = self._session
            self._phase = RELEASING
            return event

    # --------------------------------------------------------------- monitor side
    def request_release(self, now: float, source: str = "") -> Validation:
        """Validate a release request and record it. NEVER releases anything.

        Takes only this object's lock, so it cannot be blocked by, and cannot
        block, the task holding the execution lock.
        """
        with self._lock:
            if not self._holding:
                return Validation(False, "rejected: no object is recorded as held")
            # Duplicate is checked before phase: once a release has been
            # accepted for this session the phase has already advanced, and
            # "duplicate" is the more useful diagnostic than "not waiting".
            if self._consumed_session == self._session:
                return Validation(
                    False, "rejected: duplicate; a release was already accepted for this "
                           "handover session (%d)" % self._session)
            if self._pending is not None:
                return Validation(
                    False, "rejected: duplicate; release request %d is already pending"
                           % self._pending.sequence)
            if self._phase != AWAITING_PULL:
                return Validation(
                    False,
                    "rejected: not waiting for a handover (phase %s; the detector must "
                    "only be armed in %s)" % (self._phase, AWAITING_PULL),
                )
            if not self._release_authorized:
                return Validation(
                    False,
                    "rejected: release authorization absent (detection-only mode, or "
                    "motion not enabled)",
                )
            age = float(now) - self._awaiting_since
            if self._max_awaiting_sec > 0.0 and age > self._max_awaiting_sec:
                return Validation(
                    False,
                    "rejected: stale; this handover has been awaiting a pull for %.1fs "
                    "(limit %.1fs)" % (age, self._max_awaiting_sec),
                )

            self._sequence += 1
            self._pending = PendingRelease(
                sequence=self._sequence, session=self._session, side=self._side,
                requested_at=float(now), source=source,
            )
            return Validation(
                True,
                "accepted: validated pull recorded as event %d for the %s hand holding "
                "%r; the owning task will perform the release"
                % (self._sequence, self._side, self._label),
                sequence=self._sequence,
            )

    # ------------------------------------------------------------------ read-only
    def snapshot(self) -> Snapshot:
        with self._lock:
            return Snapshot(
                phase=self._phase, side=self._side, label=self._label,
                session=self._session, holding=self._holding,
                release_authorized=self._release_authorized,
                pending=self._pending is not None,
                consumed_this_session=self._consumed_session == self._session,
                awaiting_since=self._awaiting_since,
            )

    @property
    def phase(self) -> str:
        return self.snapshot().phase

    @property
    def holding(self) -> bool:
        return self.snapshot().holding
