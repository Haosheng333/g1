"""Confirmation and dispatch state machine. No ROS, no Vosk, no I/O.

Owns exactly one thing: deciding whether a recognised command may turn into a
call through `orchestrator_client`, and recording the outcome. The clock and
the route client are both injected (same pattern as orchestrator's own
Backend injection) so this is fully testable without real time or a real
service.

State is intentionally small and explicit:
  IDLE            -- nothing pending, nothing in flight
  PENDING_CONFIRM -- a task-start command was heard, awaiting "确认执行"
  BUSY            -- confirmation produced a single-use DispatchAction and
                     the route call is ready or in flight on a worker

Commands that need confirmation: "grasp", "deliver", "fetch_deliver" (they
can move the robot and cost real time/wear to undo). "status" is handled by
the caller without going through this state machine -- it is a read. "cancel"
drops a pending command here; it has no route to confirm into.

Two confirmation modes:
  confirm        a pending command dispatches only on an explicit "confirm"
                 inside the window; silence lets it expire (default).
  cancel_window  g1_intellect task_confirm_node's behaviour: the window is a
                 chance to say "cancel"; when it closes the command DISPATCHES.
                 Selected by `auto_dispatch_after_window=True`.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional, Protocol


class State(Enum):
    IDLE = "IDLE"
    PENDING_CONFIRM = "PENDING_CONFIRM"
    BUSY = "BUSY"


# Commands that require a separate confirmation before they can dispatch.
CONFIRMABLE_COMMANDS = ("grasp", "deliver", "fetch_deliver")


@dataclass(frozen=True)
class RouteCallResult:
    """Mirrors the fields of orchestrator/RunRoute.srv's response that this
    package needs to report back; see orchestrator_client for the real
    translation from a rospy service response."""

    success: bool
    message: str
    blockers: tuple = ()
    dry_run: bool = True
    motion_issued: bool = False


class RouteClient(Protocol):
    """What dispatch.py needs from the orchestrator side. Implemented by a
    real rospy ServiceProxy wrapper (orchestrator_client.RosRouteClient) and,
    for tests and the default safe mode, by RecordingDryRunClient below."""

    def call_route(self, command_id: str, target: str, dry_run: bool, **slots) -> RouteCallResult: ...


class RecordingDryRunClient:
    """Default backend: records the request it WOULD have sent and makes no
    task-execution service call of any kind. Mirrors orchestrator's own
    DryRunBackend -- dry-run safety is a property of which object got
    injected, not a flag re-checked at every call site."""

    def __init__(self) -> None:
        self.calls: list = []

    def call_route(self, command_id: str, target: str, dry_run: bool, **slots) -> RouteCallResult:
        call = {"command_id": command_id, "target": target, "dry_run": True}
        call.update(slots)
        self.calls.append(call)
        extra = "".join(f" {k}={v!r}" for k, v in sorted(slots.items()))
        return RouteCallResult(
            success=True,
            message=f"dry run: would call route for {command_id!r} with target={target!r}{extra}",
            dry_run=True,
        )


@dataclass
class PendingCommand:
    command_id: str
    target: str
    issued_at: float
    expires_at: float
    #: fetch_deliver: destination and label, forwarded to the route call.
    slots: dict = field(default_factory=dict)


@dataclass
class DispatchEvent:
    """What on_transcript/on_confirm_tick return, for the node to speak/log."""

    kind: str  # awaiting_confirmation | dispatch_ready | dispatched | rejected | expired | ignored | cancelled | unavailable | fault
    detail: str = ""
    route_result: Optional[RouteCallResult] = None
    dispatch_action: Optional["DispatchAction"] = None


@dataclass(frozen=True)
class DispatchAction:
    """An immutable, single-use route request created by confirmation.

    Creating an action moves the dispatcher to BUSY without making a blocking
    service call.  The ROS node can therefore preserve transcript order, then
    execute the action on a worker while later commands observe BUSY.
    """

    action_id: int
    command_id: str
    target: str
    slots: dict = field(default_factory=dict)


class Dispatcher:
    def __init__(
        self,
        route_client: RouteClient,
        clock: Callable[[], float],
        confirmation_window_sec: float = 8.0,
        target_resolver: Optional[Callable[[str], Optional[str]]] = None,
        auto_dispatch_after_window: bool = False,
    ) -> None:
        self._client = route_client
        self._clock = clock
        self._window = float(confirmation_window_sec)
        # cancel_window mode: when the window closes with no cancel, the
        # pending command dispatches instead of expiring.
        self._auto_dispatch = bool(auto_dispatch_after_window)
        # Maps a command_id ("grasp"/"deliver") to its configured target, or
        # None if unset. Kept injectable so tests don't need a YAML file.
        self._target_resolver = target_resolver or (lambda command_id: None)

        self.state = State.IDLE
        self._pending: Optional[PendingCommand] = None
        self.last_result: Optional[RouteCallResult] = None
        self.last_route_command: str = ""
        self._lock = threading.RLock()
        self._next_action_id = 1
        self._active_action_id: Optional[int] = None
        self._action_started = False

    # -- introspection, for tests and the node's status reporting -----------
    @property
    def pending(self) -> Optional[PendingCommand]:
        with self._lock:
            return self._pending

    def _expire_if_due(self) -> bool:
        if self._pending is not None and self._clock() >= self._pending.expires_at:
            self._pending = None
            self.state = State.IDLE
            return True
        return False

    def poll_expiry(self) -> Optional[DispatchEvent]:
        """Expire a pending command without waiting for another utterance.

        In cancel_window mode a due command is DISPATCHED instead: the returned
        event carries the single-use action, exactly as a spoken confirm would.
        """
        with self._lock:
            if self._auto_dispatch and self._pending is not None \
                    and self._clock() >= self._pending.expires_at:
                event = self._start_action(self._pending)
                return DispatchEvent(
                    "dispatch_ready",
                    "cancel window closed with no cancel; " + event.detail,
                    dispatch_action=event.dispatch_action)
            if not self._expire_if_due():
                return None
            return DispatchEvent("expired", "pending command expired without dispatch")

    def handle_fault(self, detail: str) -> DispatchEvent:
        """Clear confirmation state after an audio/input fault.

        A route already in flight remains BUSY because voice control has no
        cancellation endpoint and must not pretend the hardware stopped.
        """
        with self._lock:
            had_pending = self._pending is not None
            self._pending = None
            if self.state is not State.BUSY:
                self.state = State.IDLE
            suffix = "; pending command cleared" if had_pending else ""
            if self.state is State.BUSY:
                suffix += "; route already in flight was not cancelled"
            return DispatchEvent("fault", f"{detail}{suffix}")

    # -- the one entry point for a matched command ---------------------------
    def handle_command(self, command_id: str, slots: Optional[dict] = None) -> DispatchEvent:
        with self._lock:
            self._expire_if_due()

            if command_id in CONFIRMABLE_COMMANDS:
                return self._handle_task_start(command_id, dict(slots or {}))
            if command_id == "confirm":
                return self._handle_confirm()
            if command_id == "cancel":
                return self._handle_cancel()
            raise ValueError(
                f"handle_command got {command_id!r}; 'status' is not dispatched here -- "
                "the node reads it directly from the latched topic"
            )

    def _handle_task_start(self, command_id: str, slots: dict) -> DispatchEvent:
        if self.state is State.BUSY:
            return DispatchEvent("rejected", f"busy executing {self.last_route_command!r}; try again after it finishes")

        # The slot grammar carries its own target (the pickup waypoint it
        # parsed); the fixed phrases take the configured one.
        target = slots.pop("target", None) or self._target_resolver(command_id)
        if target is None:
            return DispatchEvent(
                "rejected",
                f"no target configured for {command_id!r}; refusing rather than inventing a destination/label",
            )

        now = self._clock()
        # A fresh task-start utterance REPLACES any not-yet-confirmed pending
        # command (nothing has executed yet, so there is nothing to protect
        # by keeping the stale one) rather than stacking requests.
        self._pending = PendingCommand(command_id=command_id, target=target, issued_at=now,
                                       expires_at=now + self._window, slots=dict(slots))
        self.state = State.PENDING_CONFIRM
        described = f"target={target!r}" + "".join(f" {k}={v!r}" for k, v in sorted(slots.items()))
        if self._auto_dispatch:
            return DispatchEvent(
                "awaiting_confirmation",
                f"heard {command_id!r} ({described}); starts in {self._window:.0f}s unless "
                f"\"取消任务\" / \"cancel task\" is said",
            )
        return DispatchEvent(
            "awaiting_confirmation",
            f"heard {command_id!r} ({described}); say “确认执行” / \"confirm\" within {self._window:.0f}s",
        )

    def _handle_confirm(self) -> DispatchEvent:
        if self.state is State.BUSY:
            return DispatchEvent("ignored", "already executing a command; confirm has nothing to apply to")
        if self._pending is None:
            return DispatchEvent("ignored", "nothing pending to confirm")

        return self._start_action(self._pending)

    def _start_action(self, pending: PendingCommand) -> DispatchEvent:
        """Turn the pending command into a single-use action; state -> BUSY."""
        self._pending = None
        self.state = State.BUSY
        self.last_route_command = pending.command_id
        action = DispatchAction(
            action_id=self._next_action_id,
            command_id=pending.command_id,
            target=pending.target,
            slots=dict(pending.slots),
        )
        self._next_action_id += 1
        self._active_action_id = action.action_id
        self._action_started = False
        return DispatchEvent(
            "dispatch_ready",
            f"confirmed {pending.command_id!r} (target={pending.target!r}); dispatch starting",
            dispatch_action=action,
        )

    def execute_action(self, action: DispatchAction) -> DispatchEvent:
        """Execute one confirmed action without holding the state lock."""
        with self._lock:
            if self.state is not State.BUSY or self._active_action_id != action.action_id:
                return DispatchEvent("rejected", "dispatch action is stale or no longer active")
            if self._action_started:
                return DispatchEvent("rejected", "dispatch action has already been started")
            self._action_started = True

        try:
            result = self._client.call_route(action.command_id, action.target, dry_run=False,
                                             **action.slots)
        except Exception as exc:  # noqa: BLE001 - a raising call is a failed call, not a crash
            result = RouteCallResult(success=False, message=f"{type(exc).__name__}: {exc}")

        with self._lock:
            self.last_result = result
            if self._active_action_id == action.action_id:
                self._active_action_id = None
                self._action_started = False
                self.state = State.IDLE
        return DispatchEvent("dispatched", result.message, route_result=result)

    def _handle_cancel(self) -> DispatchEvent:
        # The currently declared adapter has no cancellation mapping. Re-check
        # the target workspace before deployment. Locally, an unconfirmed
        # pending command can always be dropped because nothing has executed.
        had_pending = self._pending is not None
        self._pending = None
        if self.state is not State.BUSY:
            self.state = State.IDLE
        if self.state is State.BUSY:
            return DispatchEvent(
                "unavailable",
                "no cancellation service exists on the orchestrator (gap, not implemented here); "
                "a route already in flight cannot be stopped by voice",
            )
        if had_pending:
            return DispatchEvent("cancelled", "pending command cleared; nothing was dispatched")
        return DispatchEvent("cancelled", "nothing was pending")
