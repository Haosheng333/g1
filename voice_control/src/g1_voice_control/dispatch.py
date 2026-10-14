"""Confirmation and dispatch state machine. No ROS, no Vosk, no I/O.

Owns exactly one thing: deciding whether a recognised command may turn into a
call through `orchestrator_client`, and recording the outcome. The clock and
the route client are both injected (same pattern as orchestrator's own
Backend injection) so this is fully testable without real time or a real
service.

State is intentionally small and explicit:
  IDLE            -- nothing pending, nothing in flight
  PENDING_CONFIRM -- a task-start command was heard, awaiting "确认执行"
  BUSY            -- a route call is in flight (dispatch is synchronous from
                     this object's point of view; the node wraps the actual
                     call in a background thread so the mic loop stays live)

Commands that need confirmation: "grasp", "deliver" (they can move the robot
and cost real time/wear to undo). "status" and "cancel" are handled by the
caller without going through this state machine -- status is a read, and
cancel has no route to confirm into.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional, Protocol


class State(Enum):
    IDLE = "IDLE"
    PENDING_CONFIRM = "PENDING_CONFIRM"
    BUSY = "BUSY"


# Commands that require a separate confirmation before they can dispatch.
CONFIRMABLE_COMMANDS = ("grasp", "deliver")


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

    def call_route(self, command_id: str, target: str, dry_run: bool) -> RouteCallResult: ...


class RecordingDryRunClient:
    """Default backend: records the request it WOULD have sent and makes no
    task-execution service call of any kind. Mirrors orchestrator's own
    DryRunBackend -- dry-run safety is a property of which object got
    injected, not a flag re-checked at every call site."""

    def __init__(self) -> None:
        self.calls: list = []

    def call_route(self, command_id: str, target: str, dry_run: bool) -> RouteCallResult:
        self.calls.append({"command_id": command_id, "target": target, "dry_run": dry_run})
        return RouteCallResult(
            success=True,
            message=f"dry run: would call route for {command_id!r} with target={target!r}",
            dry_run=True,
        )


@dataclass
class PendingCommand:
    command_id: str
    target: str
    issued_at: float
    expires_at: float


@dataclass
class DispatchEvent:
    """What on_transcript/on_confirm_tick return, for the node to speak/log."""

    kind: str  # "awaiting_confirmation" | "dispatched" | "rejected" | "expired" | "ignored" | "cancelled"
    detail: str = ""
    route_result: Optional[RouteCallResult] = None


class Dispatcher:
    def __init__(
        self,
        route_client: RouteClient,
        clock: Callable[[], float],
        confirmation_window_sec: float = 8.0,
        target_resolver: Optional[Callable[[str], Optional[str]]] = None,
    ) -> None:
        self._client = route_client
        self._clock = clock
        self._window = float(confirmation_window_sec)
        # Maps a command_id ("grasp"/"deliver") to its configured target, or
        # None if unset. Kept injectable so tests don't need a YAML file.
        self._target_resolver = target_resolver or (lambda command_id: None)

        self.state = State.IDLE
        self._pending: Optional[PendingCommand] = None
        self.last_result: Optional[RouteCallResult] = None
        self.last_route_command: str = ""

    # -- introspection, for tests and the node's status reporting -----------
    @property
    def pending(self) -> Optional[PendingCommand]:
        return self._pending

    def _expire_if_due(self) -> bool:
        if self._pending is not None and self._clock() >= self._pending.expires_at:
            self._pending = None
            self.state = State.IDLE
            return True
        return False

    # -- the one entry point for a matched command ---------------------------
    def handle_command(self, command_id: str) -> DispatchEvent:
        self._expire_if_due()

        if command_id in CONFIRMABLE_COMMANDS:
            return self._handle_task_start(command_id)
        if command_id == "confirm":
            return self._handle_confirm()
        if command_id == "cancel":
            return self._handle_cancel()
        raise ValueError(
            f"handle_command got {command_id!r}; 'status' is not dispatched here -- "
            "the node reads it directly from the latched topic"
        )

    def _handle_task_start(self, command_id: str) -> DispatchEvent:
        if self.state is State.BUSY:
            return DispatchEvent("rejected", f"busy executing {self.last_route_command!r}; try again after it finishes")

        target = self._target_resolver(command_id)
        if target is None:
            return DispatchEvent(
                "rejected",
                f"no target configured for {command_id!r}; refusing rather than inventing a destination/label",
            )

        now = self._clock()
        # A fresh task-start utterance REPLACES any not-yet-confirmed pending
        # command (nothing has executed yet, so there is nothing to protect
        # by keeping the stale one) rather than stacking requests.
        self._pending = PendingCommand(command_id=command_id, target=target, issued_at=now, expires_at=now + self._window)
        self.state = State.PENDING_CONFIRM
        return DispatchEvent(
            "awaiting_confirmation",
            f"heard {command_id!r} (target={target!r}); say “确认执行” / \"confirm\" within {self._window:.0f}s",
        )

    def _handle_confirm(self) -> DispatchEvent:
        if self.state is State.BUSY:
            return DispatchEvent("ignored", "already executing a command; confirm has nothing to apply to")
        if self._pending is None:
            return DispatchEvent("ignored", "nothing pending to confirm")

        pending = self._pending
        self._pending = None
        self.state = State.BUSY
        self.last_route_command = pending.command_id
        try:
            # Synchronous from here; the node runs handle_command() for
            # "confirm" on a worker thread so the mic loop keeps listening.
            result = self._client.call_route(pending.command_id, pending.target, dry_run=False)
        except Exception as exc:  # noqa: BLE001 - a raising call is a failed call, not a crash
            result = RouteCallResult(success=False, message=f"{type(exc).__name__}: {exc}")
        self.last_result = result
        self.state = State.IDLE
        return DispatchEvent("dispatched", result.message, route_result=result)

    def _handle_cancel(self) -> DispatchEvent:
        # The orchestrator exposes no cancellation service (confirmed against
        # orchestrator/README.md's interface table and sequencer.py's
        # Backend): there is no in-flight route this package can abort. What
        # it CAN safely do locally is drop an unconfirmed pending command,
        # since nothing has executed yet.
        had_pending = self._pending is not None
        self._pending = None
        if self.state is not State.BUSY:
            self.state = State.IDLE
        if self.state is State.BUSY:
            return DispatchEvent(
                "cancelled",
                "no cancellation service exists on the orchestrator (gap, not implemented here); "
                "a route already in flight cannot be stopped by voice",
            )
        if had_pending:
            return DispatchEvent("cancelled", "pending command cleared; nothing was dispatched")
        return DispatchEvent("cancelled", "nothing was pending")
