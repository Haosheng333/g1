"""The voice module's side of the orchestrator contract. ROS-dependent.

The orchestrator is the top of the system and treats this package as one of
its inputs, the way g1_intellect's main_process subscribes to g1_voice_intent's
/voice/intent. So nothing here calls an orchestrator service. A confirmed task
is PUBLISHED as g1_voice_control/TaskCommand on /voice/command; the orchestrator
subscribes, runs the route under its own lock, and publishes
g1_voice_control/TaskResult on /orchestrator/command_result with the same
`seq`. `TopicCommandClient` wraps that round trip behind dispatch.RouteClient,
so the confirm / busy state machine is unchanged.

This package depends on no other DIP package: the message types are its own.
"""

from __future__ import annotations

import threading
from typing import Optional

try:
    import rospy
except ImportError as exc:  # pragma: no cover - exercised only off-robot
    rospy = None
    _ROSPY_IMPORT_ERROR = exc
else:
    _ROSPY_IMPORT_ERROR = None

from g1_voice_control.dispatch import RouteCallResult

#: Voice command id -> orchestrator route name. The orchestrator refuses
#: anything it does not know, so an unmapped command is refused here first.
COMMAND_TO_ROUTE_NAME = {
    "grasp": "grasp",
    "deliver": "deliver",
    "fetch_deliver": "fetch_deliver",
}
COMMAND_TOPIC = "/voice/command"
RESULT_TOPIC = "/orchestrator/command_result"
STATUS_TOPIC = "/orchestrator/status"


def _require_rospy() -> None:
    if rospy is None:
        raise RuntimeError(
            "rospy is not importable (are you running this outside a ROS "
            f"environment?): {_ROSPY_IMPORT_ERROR}"
        )


class TopicCommandClient:
    """Implements dispatch.RouteClient over /voice/command + /orchestrator/command_result.

    `call_route` publishes one TaskCommand and blocks until the TaskResult with
    the same `seq` arrives, so dispatch.py's BUSY state covers the whole route,
    exactly as it did over the service. A route can run for minutes, so there is
    no default deadline; `result_timeout_s` adds one for tests and tooling.
    `subscriber_wait_s` bounds how long to wait for the orchestrator to be
    subscribed before publishing, so a command is never dropped on the floor.
    """

    def __init__(self, result_timeout_s: Optional[float] = None, subscriber_wait_s: float = 10.0,
                 command_topic: str = COMMAND_TOPIC, result_topic: str = RESULT_TOPIC) -> None:
        _require_rospy()
        from g1_voice_control.msg import TaskCommand, TaskResult

        self._TaskCommand = TaskCommand
        self._result_timeout_s = result_timeout_s
        self._subscriber_wait_s = float(subscriber_wait_s)
        self._lock = threading.Lock()
        self._seq = 0
        self._pending: dict = {}          # seq -> [Event, TaskResult | None]
        self._pub = rospy.Publisher(command_topic, TaskCommand, queue_size=5)
        self._sub = rospy.Subscriber(result_topic, TaskResult, self._on_result, queue_size=10)
        self.command_topic = command_topic
        self.result_topic = result_topic

    def _on_result(self, msg) -> None:
        with self._lock:
            slot = self._pending.get(int(msg.seq))
            if slot is None:
                return
            slot[1] = msg
            slot[0].set()

    def _wait_for_subscriber(self) -> bool:
        deadline = rospy.get_time() + self._subscriber_wait_s
        while self._pub.get_num_connections() == 0:
            if rospy.is_shutdown() or rospy.get_time() >= deadline:
                return False
            rospy.sleep(0.1)
        return True

    def call_route(self, command_id: str, target: str, dry_run: bool,
                   destination: str = "", label: str = "", summary: str = "",
                   transcript: str = "") -> RouteCallResult:
        if command_id not in COMMAND_TO_ROUTE_NAME:
            raise ValueError(f"no orchestrator route mapped for command {command_id!r}")
        if not self._wait_for_subscriber():
            return RouteCallResult(
                success=False,
                message=f"nobody subscribes to {self.command_topic}; is the orchestrator running?")

        with self._lock:
            self._seq += 1
            seq = self._seq
            event = threading.Event()
            self._pending[seq] = [event, None]

        msg = self._TaskCommand(
            seq=seq, command_id=COMMAND_TO_ROUTE_NAME[command_id], target=target or "",
            destination=destination or "", label=label or "", summary=summary or "",
            transcript=transcript or "")
        self._pub.publish(msg)

        try:
            while not event.is_set():
                if rospy.is_shutdown():
                    return RouteCallResult(success=False, message="shutdown while awaiting the result")
                if self._result_timeout_s is not None and not event.wait(self._result_timeout_s):
                    return RouteCallResult(
                        success=False,
                        message=f"no result for command {seq} within {self._result_timeout_s:.0f}s")
                event.wait(0.5)
        finally:
            with self._lock:
                slot = self._pending.pop(seq, None)
        result = slot[1] if slot else None
        if result is None:
            return RouteCallResult(success=False, message="result lost")
        return RouteCallResult(
            success=bool(result.success),
            message=result.message,
            blockers=tuple(result.blockers),
            dry_run=bool(result.dry_run),
            motion_issued=bool(result.motion_issued),
        )


class StatusReader:
    """Caches the latest /orchestrator/status message (std_msgs/String,
    latched). Subscribes once; `status` is deliberately a read, so this
    never shares the Dispatcher's confirm/busy state machine."""

    def __init__(self) -> None:
        _require_rospy()
        from std_msgs.msg import String

        self._latest: Optional[str] = None
        self._lock = threading.Lock()
        self._sub = rospy.Subscriber(STATUS_TOPIC, String, self._on_status)

    def _on_status(self, msg) -> None:
        with self._lock:
            self._latest = msg.data

    def latest(self) -> str:
        with self._lock:
            if self._latest is None:
                return "no status received yet (orchestrator node may not be up)"
            return self._latest
