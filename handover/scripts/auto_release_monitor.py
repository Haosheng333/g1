#!/usr/bin/env python3
"""Auto-release handover monitor.

Watches a tactile signal for the moment a human takes the held object, and
then asks the ORCHESTRATOR to run the release. It never commands the arm and
never opens the hand itself: detection and actuation are deliberately split so
a sensor glitch cannot drop an object on its own.

    /auto_release_monitor/arm      std_srvs/Trigger   start baselining
    /auto_release_monitor/disarm   std_srvs/Trigger   back to IDLE, always safe
    /auto_release_monitor/state    std_msgs/String    (latched) state + reason
    /auto_release_monitor/ready    std_msgs/Bool      (latched) can it be armed

On a verified detection it calls, exactly once:

    /orchestrator/run_auto_release   orchestrator/RunRoute

The state machine lives in handover.release_monitor and knows nothing about
ROS; the tactile message -> scalar step lives in handover.tactile_adapter and
is DISABLED by default because no tactile topic exists in this workspace yet
(hand_control reads the Revo2 over serial and publishes no sensor topic). While
it is disabled, arm() refuses with NOT_READY and names what the hand owner has
to confirm, so nothing can fire off a guessed schema or a guessed threshold.
"""
from __future__ import annotations

import threading

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from handover import tactile_adapter as ta
from handover.release_monitor import (
    IDLE,
    MonitorConfig,
    ReleaseMonitor,
    TRIGGERED,
)

#: Service the monitor asks to perform the release. Never a hand/arm service.
RELEASE_ROUTE_SERVICE = "/orchestrator/run_auto_release"


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _optional_number(name):
    """A hardware threshold must be set explicitly; empty/absent stays None."""
    value = rospy.get_param("~" + name, None)
    if value is None or value == "":
        return None
    return float(value)


def _optional_string(name):
    value = rospy.get_param("~" + name, None)
    if value is None or value == "":
        return None
    return str(value)


class AutoReleaseMonitorNode:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sample = None
        self._sample_stamp = None
        self._fired = False

        self.adapter = ta.AdapterConfig(
            enabled=bool(_param("tactile/enabled", False)),
            topic=str(_param("tactile/topic", "")),
            message_type=str(_param("tactile/message_type", "")),
            value_field=str(_param("tactile/value_field", "")),
            reduction=str(_param("tactile/reduction", "mean")),
        )
        self.config = MonitorConfig(
            release_delta=_optional_number("release_delta"),
            direction=_optional_string("direction"),
            baseline_samples=int(_param("baseline_samples", 20)),
            baseline_timeout_s=float(_param("baseline_timeout_s", 5.0)),
            debounce_samples=int(_param("debounce_samples", 3)),
            min_hold_s=float(_param("min_hold_s", 0.20)),
            sample_timeout_s=float(_param("sample_timeout_s", 0.50)),
            monitor_timeout_s=float(_param("monitor_timeout_s", 120.0)),
        )
        self.monitor = ReleaseMonitor(config=self.config)
        self.rate_hz = float(_param("rate", 20.0))

        self.state_pub = rospy.Publisher(
            "/auto_release_monitor/state", String, queue_size=1, latch=True
        )
        self.ready_pub = rospy.Publisher(
            "/auto_release_monitor/ready", Bool, queue_size=1, latch=True
        )

        self._subscribe_tactile()
        self._publish()

    # ------------------------------------------------------------------ blockers
    def blockers(self) -> list:
        """Everything standing between this node and a usable armed monitor."""
        return self.adapter.validate() + self.config.validate()

    @property
    def ready(self) -> bool:
        return not self.blockers()

    def _publish(self) -> None:
        if self.ready:
            text = "%s: %s" % (self.monitor.state, self.monitor.last_reason)
        else:
            text = "%s: NOT_READY: %s" % (IDLE, "; ".join(self.blockers()))
        self.state_pub.publish(String(data=text))
        self.ready_pub.publish(Bool(data=self.ready))

    # ------------------------------------------------------------------ tactile
    def _subscribe_tactile(self) -> None:
        """Subscribe only when the adapter is fully configured and enabled."""
        problems = self.adapter.validate()
        if problems:
            rospy.logwarn("[auto_release_monitor] no tactile input: %s", "; ".join(problems))
            return
        try:
            message_class = ta.resolve_message_class(self.adapter.message_type)
        except (ImportError, ValueError) as exc:
            rospy.logerr(
                "[auto_release_monitor] cannot load tactile message type %r: %s",
                self.adapter.message_type,
                exc,
            )
            return
        rospy.Subscriber(self.adapter.topic, message_class, self._on_tactile, queue_size=5)
        rospy.loginfo(
            "[auto_release_monitor] tactile input %s (%s), field %r, reduction %s",
            self.adapter.topic,
            self.adapter.message_type,
            self.adapter.value_field,
            self.adapter.reduction,
        )

    def _on_tactile(self, msg) -> None:
        try:
            value = ta.extract_scalar(msg, self.adapter)
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logerr_throttle(
                5.0, "[auto_release_monitor] cannot read tactile sample: %s" % (exc,)
            )
            return
        with self._lock:
            self._sample = value
            self._sample_stamp = rospy.Time.now().to_sec()

    # ------------------------------------------------------------------ services
    def handle_arm(self, _req) -> TriggerResponse:
        blockers = self.blockers()
        if blockers:
            message = "NOT_READY: " + "; ".join(blockers)
            rospy.logwarn("[auto_release_monitor] arm refused: %s", message)
            self._publish()
            return TriggerResponse(success=False, message=message)

        with self._lock:
            self._fired = False
            self._sample = None
            self._sample_stamp = None
            outcome = self.monitor.arm(rospy.Time.now().to_sec())
        rospy.loginfo("[auto_release_monitor] arm: %s", outcome.reason)
        self._publish()
        return TriggerResponse(success=outcome.ok, message=outcome.reason)

    def handle_disarm(self, _req) -> TriggerResponse:
        with self._lock:
            outcome = self.monitor.disarm(rospy.Time.now().to_sec(), "disarmed by request")
            self._sample = None
            self._sample_stamp = None
            self._fired = False
        rospy.logwarn("[auto_release_monitor] disarm: %s", outcome.reason)
        self._publish()
        return TriggerResponse(success=True, message=outcome.reason)

    # ------------------------------------------------------------------ main loop
    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self) -> None:
        now = rospy.Time.now().to_sec()
        with self._lock:
            if not self.monitor.armed:
                return
            sample = self._sample
            stamp = self._sample_stamp
            # A sample older than the staleness window is not offered at all,
            # so the state machine sees "no data" rather than an old value.
            if stamp is None or (now - stamp) > self.config.sample_timeout_s:
                sample = None
            event = self.monitor.update(now, sample)

        if event.wait:
            rospy.logwarn_throttle(2.0, "[auto_release_monitor] %s" % (event.reason,))
        self._publish()

        if event.trigger and not self._fired:
            self._fired = True
            rospy.loginfo("[auto_release_monitor] %s", event.reason)
            self._request_release(event.reason)

    def _request_release(self, reason: str) -> None:
        """Ask the orchestrator to run the release. Never actuate here."""
        try:
            # Imported lazily so this node still starts when orchestrator
            # messages have not been generated yet.
            from orchestrator.srv import RunRoute
        except ImportError as exc:
            rospy.logerr(
                "[auto_release_monitor] detected a release but orchestrator messages are "
                "unavailable (%s); NOT actuating anything. Build the orchestrator package.",
                exc,
            )
            return
        try:
            rospy.wait_for_service(RELEASE_ROUTE_SERVICE, timeout=5.0)
            call = rospy.ServiceProxy(RELEASE_ROUTE_SERVICE, RunRoute)
            response = call(route="auto_release", target="", start_step="", dry_run=False)
            if response.success:
                rospy.loginfo("[auto_release_monitor] %s accepted", RELEASE_ROUTE_SERVICE)
            else:
                rospy.logerr(
                    "[auto_release_monitor] %s refused: %s",
                    RELEASE_ROUTE_SERVICE,
                    response.message,
                )
        except rospy.ROSException as exc:
            rospy.logerr(
                "[auto_release_monitor] %s unavailable (%s); nothing was actuated",
                RELEASE_ROUTE_SERVICE,
                exc,
            )
        except rospy.ServiceException as exc:
            rospy.logerr("[auto_release_monitor] %s failed: %s", RELEASE_ROUTE_SERVICE, exc)


def main() -> None:
    rospy.init_node("auto_release_monitor")
    node = AutoReleaseMonitorNode()
    rospy.Service("/auto_release_monitor/arm", Trigger, node.handle_arm)
    rospy.Service("/auto_release_monitor/disarm", Trigger, node.handle_disarm)
    if node.ready:
        rospy.loginfo("[auto_release_monitor] ready; call /auto_release_monitor/arm to start")
    else:
        rospy.logwarn(
            "[auto_release_monitor] NOT_READY, arming will be refused: %s",
            "; ".join(node.blockers()),
        )
    node.spin()


if __name__ == "__main__":
    main()
