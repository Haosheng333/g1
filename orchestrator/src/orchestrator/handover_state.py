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
import uuid
from dataclasses import dataclass, field

# --- task phases ---------------------------------------------------------------------

#: Nothing held, no handover in progress.
IDLE = "IDLE"
#: Holding an object; the arm is moving to the handover pose. The detector must
#: NOT be armed in this phase.
MOVING_TO_POSE = "MOVING_TO_POSE"
#: The token has been issued and the monitor is being armed, but the task has
#: not yet entered AWAITING_PULL. This phase exists so the handshake window is
#: not a hole: the monitor can in principle detect and send a request between
#: its own arm handler returning and the orchestrator calling
#: mark_awaiting_pull. A request with the CORRECT token is accepted and queued
#: here, so the only event is neither lost nor acted on early -- await_pull
#: consumes it a moment later. If arming then fails or the echo mismatches, the
#: task abandons and the queued event is dropped without any release.
ARMING = "ARMING"
#: At the pose, settled, detector armed, waiting for a human pull. This is the
#: ONLY phase in which a release request can be accepted.
AWAITING_PULL = "AWAITING_PULL"
#: A validated pull was consumed; the owning task is releasing.
RELEASING = "RELEASING"

PHASES = (IDLE, MOVING_TO_POSE, ARMING, AWAITING_PULL, RELEASING)

#: Phases in which a correctly-tokened release request may be queued.
ACCEPTING_PHASES = (ARMING, AWAITING_PULL)

# --- object states -------------------------------------------------------------------
#
# Deliberately NOT a boolean. hand_control's `release` preset sets neither
# require_settle nor expect_contact, so /hand_{side}/release returns success as
# soon as the open command is accepted -- even if the fingers never moved
# (hand_node._execute: "release commanded, did not settle ... expected if
# holding an object"). A successful response is therefore an actuation
# acknowledgement, NOT evidence that the object transferred.

#: Nothing is held.
NONE = "NONE"
#: An object is held, confirmed by a successful grasp sequence.
HELD = "HELD"
#: The open command was ACCEPTED by hand_control. Nothing more: the fingers may
#: not have moved, and nothing has observed the object leaving. This is the
#: strongest state the current hand contract can justify.
RELEASE_COMMANDED = "RELEASE_COMMANDED"
#: The fingers were CONFIRMED to have reached the open target, by evidence the
#: orchestrator can actually check.
#:
#: UNREACHABLE TODAY, deliberately. /hand_{side}/release returns only
#: `success` and a human-readable `message`. hand_control does compute settling
#: internally (revo_hand_link.wait_until_settled polls fresh read_positions()
#: against the target within settle_tolerance, deadline settle_timeout_s), but
#: it does not expose the result, the `release` preset leaves require_settle
#: absent (so it defaults false), and _execute therefore does NOT fail on a
#: settling timeout -- it returns success with "release commanded, did not
#: settle". So nothing in the response distinguishes settled from not.
#:
#: An orchestrator-side configuration flag must NOT be used to assert this: it
#: would be this package claiming a fact about another package's YAML that it
#: cannot observe. Reaching this state needs a hand_control interface change
#: (see mark_release_settled).
#:
#: Even once reachable this is NOT evidence of transfer: fingers can open
#: around nothing, and an object can stay stuck to the hand.
RELEASE_SETTLED = "RELEASE_SETTLED"
#: The release failed, raised, or its outcome is unknown. The object may still
#: be in the hand or falling. Retraction is REFUSED and a human must look.
UNCERTAIN = "UNCERTAIN"

OBJECT_STATES = (NONE, HELD, RELEASE_COMMANDED, RELEASE_SETTLED, UNCERTAIN)

#: Three different things, deliberately not collapsed:
#:   RELEASE_COMMANDED  the service said yes
#:   RELEASE_SETTLED    the fingers are confirmed open
#:   transfer verified  something observed the object leave  <- NO state here,
#:                      because nothing in this workspace can observe it. It is
#:                      reported as the separate `transfer_verified` response
#:                      field, which is always false.
RELEASED_STATES = (RELEASE_COMMANDED, RELEASE_SETTLED)


def new_session_token() -> str:
    """The ONE place a handover session token is created.

    The orchestrator calls this once per arming, hands the value to the monitor
    through ArmMonitor.srv, and the monitor echoes that same value back in its
    release request. Nothing else generates a token.
    """
    return uuid.uuid4().hex[:12]


@dataclass
class Validation:
    """Outcome of validating a release request. Never performs the release."""

    ok: bool
    reason: str = ""
    #: Monotonic id of the recorded event, when one was recorded.
    sequence: int = 0
    #: True when the request carried no session token, so its origin handover
    #: could not be established. See HandoverState.request_release.
    origin_unverified: bool = False


@dataclass
class PendingRelease:
    """A validated pull, waiting for the owning task to act on it."""

    sequence: int
    session: int
    side: str
    requested_at: float
    source: str = ""
    #: Session token the request carried, "" when it carried none.
    event_token: str = ""


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
    object_state: str = NONE
    session_token: str = ""


class HandoverState(object):
    """Task-owned handover record. Thread-safe via its OWN lock."""

    def __init__(self, max_awaiting_sec: float = 120.0, require_event_token: bool = True):
        self._lock = threading.Lock()
        self._max_awaiting_sec = float(max_awaiting_sec)
        #: When True a release request without a matching session token is
        #: REFUSED, because its origin handover cannot be established. See
        #: request_release for why receipt timestamping cannot substitute.
        self._require_event_token = bool(require_event_token)
        self._settled_evidence = ""
        self._object_state = NONE
        self._session_token = ""
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
            self._object_state = HELD
            self._release_authorized = bool(release_authorized)
            self._session += 1
            self._session_token = ""
            self._pending = None
            self._awaiting_since = 0.0

    def begin_arming(self, now: float, token: str) -> str:
        """Issue the session token and enter ARMING, BEFORE arming the monitor.

        Called first so that a request arriving during the handshake window
        carries a token this object already knows and can queue, instead of
        being rejected as "not waiting" and silently lost.
        """
        if not token:
            raise ValueError("begin_arming needs the session token")
        with self._lock:
            if not self._holding:
                raise RuntimeError("cannot arm without a held object")
            if self._object_state != HELD:
                raise RuntimeError(
                    "cannot arm with object state %s" % (self._object_state,))
            self._phase = ARMING
            self._awaiting_since = float(now)
            self._pending = None
            self._session_token = str(token)
            return self._session_token

    def mark_awaiting_pull(self, now: float, token: str) -> str:
        """Enter AWAITING_PULL with the token the monitor was armed with.

        The caller passes the token it already gave the monitor, so exactly one
        value exists for the session. Called only AFTER the monitor confirmed it
        stored that token, so there is no window in which a release could be
        validated against a monitor that is not yet armed.
        """
        if not token:
            raise ValueError("mark_awaiting_pull needs the session token given to the monitor")
        with self._lock:
            if not self._holding:
                raise RuntimeError("cannot await a pull without a held object")
            if self._object_state != HELD:
                raise RuntimeError(
                    "cannot await a pull with object state %s" % (self._object_state,))
            if self._session_token and self._session_token != str(token):
                raise ValueError(
                    "mark_awaiting_pull token %r does not match the token issued at "
                    "begin_arming (%r)" % (token, self._session_token))
            self._phase = AWAITING_PULL
            # Deliberately NOT cleared: an event queued during the ARMING
            # handshake window is the only one the monitor will send, so losing
            # it would leave the task waiting for a pull that already happened.
            self._session_token = str(token)
            if self._awaiting_since == 0.0:
                self._awaiting_since = float(now)
            return self._session_token

    def mark_releasing(self) -> None:
        with self._lock:
            self._phase = RELEASING

    def mark_release_commanded(self) -> None:
        """The open command was ACCEPTED. Transfer is not verified (see OBJECT_STATES)."""
        with self._lock:
            self._object_state = RELEASE_COMMANDED
            self._holding = False
            self._pending = None

    def mark_release_settled(self, evidence: str) -> None:
        """Record CONFIRMED finger settling. Requires checkable evidence.

        `evidence` must describe something the orchestrator observed in the
        service result -- a structured settled flag, or feedback positions it
        compared against the target itself. It may not be an orchestrator
        configuration flag, because that asserts a fact about hand_control's
        YAML that this package cannot see.

        No such evidence exists today: the release response carries only
        success and a message. Until hand_control exposes it, this stays
        unreachable and completion/retraction remain NOT_READY.

        Never implies transfer: `transfer_verified` stays false regardless.
        """
        if not evidence:
            raise ValueError(
                "mark_release_settled needs checkable evidence of settling from the "
                "hand service result. An orchestrator config flag is not evidence: the "
                "release response exposes only success and a message, and "
                "hand_control's release preset leaves require_settle absent, so a "
                "success does not mean the fingers moved.")
        with self._lock:
            self._object_state = RELEASE_SETTLED
            self._settled_evidence = str(evidence)
            self._holding = False
            self._pending = None

    def mark_release_uncertain(self, reason: str = "") -> None:
        """The release failed or its outcome is unknown: keep custody, refuse retraction."""
        with self._lock:
            self._object_state = UNCERTAIN
            self._holding = True          # still our problem until a human looks
            self._pending = None

    def mark_done(self) -> None:
        """The delivery finished and the arm is home."""
        with self._lock:
            self._phase = IDLE
            self._holding = False
            self._object_state = NONE
            self._label = ""
            self._session_token = ""
            self._pending = None

    def release_permitted(self) -> tuple:
        """(allowed, reason). The hand may only open after a validated pull.

        Guards resume: starting a route at `release_hand` must not open the
        hand just because the step name was given. Only a pull that was
        validated AND consumed in the current session authorises it.
        """
        with self._lock:
            if self._object_state == UNCERTAIN:
                return (False, "refusing to release: the outcome of a previous release is "
                               "UNCERTAIN; a human must inspect")
            if self._consumed_session != self._session:
                return (False,
                        "refusing to release: no validated pull has been consumed in this "
                        "handover session (%d). The hand opens only after a verified pull, "
                        "never because a route was resumed at this step." % self._session)
            return (True, "release authorised by consumed event in session %d" % self._session)

    def retraction_allowed(self) -> tuple:
        """(allowed, reason). Physical completion is never assumed.

        Retracting is a motion next to a person who may still have hold of the
        object, so the bar is "something confirmed the hand is open", not "a
        service returned success".

          UNCERTAIN          refused; a human must inspect
          HELD               refused; nothing was released
          RELEASE_COMMANDED  refused, with no override. A command
                             acknowledgement is not physical completion.
          RELEASE_SETTLED    allowed; settling was CONFIRMED from checkable
                             evidence. Transfer is still not verified.
          NONE               allowed; nothing is held
        """
        with self._lock:
            if self._object_state == UNCERTAIN:
                return (False,
                        "refusing to retract: the release outcome is UNCERTAIN, so the "
                        "object may still be in the hand or falling. A human must "
                        "inspect before the arm moves.")
            if self._object_state == HELD:
                return (False, "refusing to retract: the object is still recorded as held")
            if self._object_state == RELEASE_COMMANDED:
                return (False,
                        "NOT_READY: refusing to retract on a command acknowledgement "
                        "alone. /hand_{side}/release returns only success and a message; "
                        "hand_control's release preset leaves require_settle absent, so "
                        "_execute does not fail on a settling timeout and a success does "
                        "not establish that the fingers opened. There is no override: "
                        "hand_control must expose settling (a structured flag, or the "
                        "feedback positions) before the arm may move away from a person "
                        "who might still be holding the object.")
            if self._object_state == RELEASE_SETTLED:
                return (True,
                        "retraction allowed: finger settling CONFIRMED (%s). Object "
                        "transfer is still NOT verified -- nothing in this workspace "
                        "observes the object leaving the hand."
                        % (self._settled_evidence or "unspecified evidence",))
            return (True, "retraction allowed: nothing is held")

    def require_held_object(self, side: str = "") -> Validation:
        """Gate for the start of a delivery. Never invents a held object."""
        with self._lock:
            if self._object_state == UNCERTAIN:
                return Validation(False,
                                  "refusing: the previous release outcome is UNCERTAIN; a "
                                  "human must inspect before another delivery")
            if not self._holding or self._object_state != HELD:
                return Validation(False,
                                  "refusing: no object is recorded as held (object_state=%s). "
                                  "A delivery cannot start without a completed grasp."
                                  % (self._object_state,))
            if side and side != self._side:
                return Validation(False,
                                  "refusing: the held object is in the %s hand, not %s"
                                  % (self._side, side))
            return Validation(True, "holding %r in the %s hand" % (self._label, self._side))

    def abandon(self, reason: str = "") -> None:
        """Give up the handover without releasing (timeout, fault, arm failure).

        Drops any queued event and clears the session token, so a request that
        was in flight cannot be matched afterwards and no release can follow.
        """
        with self._lock:
            self._phase = MOVING_TO_POSE if self._holding else IDLE
            self._pending = None
            self._awaiting_since = 0.0
            self._session_token = ""

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
    def request_release(self, now: float, source: str = "", event_token: str = "") -> Validation:
        """Validate a release request and record it. NEVER releases anything.

        Takes only this object's lock, so it cannot be blocked by, and cannot
        block, the task holding the execution lock.

        EVENT IDENTITY. `event_token` must be the token this object handed out
        at `mark_awaiting_pull`. Without it the orchestrator cannot tell a pull
        detected during THIS handover from one detected during an earlier one
        and delivered late: the request carries no origin time, so stamping its
        RECEIPT time proves only when it arrived, not which handover it refers
        to. So when `require_event_token` is set (the default) a tokenless
        request is REFUSED rather than guessed at.
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
            if self._phase not in ACCEPTING_PHASES:
                return Validation(
                    False,
                    "rejected: not waiting for a handover (phase %s; a request is only "
                    "accepted in %s)" % (self._phase, " or ".join(ACCEPTING_PHASES)),
                )
            if not self._release_authorized:
                return Validation(
                    False,
                    "rejected: release authorization absent (detection-only mode, or "
                    "motion not enabled)",
                )
            if event_token:
                if event_token != self._session_token:
                    return Validation(
                        False,
                        "rejected: stale; token %r is not this handover's token (session "
                        "%d). An event detected during an earlier handover cannot be "
                        "accepted by this one." % (event_token, self._session),
                    )
            elif self._require_event_token:
                return Validation(
                    False,
                    "rejected: no session token, so the request's origin handover cannot "
                    "be established. Receipt time cannot substitute: it shows when the "
                    "request arrived, not which handover detected the pull. The monitor "
                    "must quote the token issued at arming; see docs/DELIVERY_FLOW.md "
                    "for the interface revision this needs.",
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
                requested_at=float(now), source=source, event_token=event_token,
            )
            return Validation(
                True,
                "accepted: validated pull recorded as event %d for the %s hand holding "
                "%r; the owning task will perform the release"
                % (self._sequence, self._side, self._label),
                sequence=self._sequence,
                origin_unverified=not event_token,
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
                object_state=self._object_state,
                session_token=self._session_token,
            )

    @property
    def phase(self) -> str:
        return self.snapshot().phase

    @property
    def holding(self) -> bool:
        return self.snapshot().holding

    @property
    def object_state(self) -> str:
        return self.snapshot().object_state

    @property
    def session_token(self) -> str:
        return self.snapshot().session_token
