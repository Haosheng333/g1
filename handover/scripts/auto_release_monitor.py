#!/usr/bin/env python3
"""Tactile handover monitor: detect a human taking the held object.

One node per hand, launched with `side:=left|right`, the same shape as
hand_control. Services and topics keep the names this branch already used, so
the orchestrator's preflight and the existing tests are unaffected; the
side-qualified aliases are added for running two hands at once.

    /auto_release_monitor/arm            handover/ArmMonitor  (side + session_token)
    /auto_release_monitor/disarm         std_srvs/Trigger
    /auto_release_monitor/<side>/arm     handover/ArmMonitor
    /auto_release_monitor/<side>/disarm  std_srvs/Trigger
    /auto_release_monitor/state          std_msgs/String  (latched) state + reason
    /auto_release_monitor/ready          std_msgs/Bool    (latched) can it be armed
    /auto_release_monitor/event          std_msgs/String  candidate / confirmed / fault

DETECTION-ONLY IS THE DEFAULT (`release_integration:=false`). In that mode this
node constructs NO release client of any kind: it cannot open the hand and
cannot ask the orchestrator to. It consumes tactile readings, runs the detector
and publishes what it sees. That is the mode to calibrate in.

With `release_integration:=true` a verified pull produces exactly ONE request to
/orchestrator/run_auto_release, which only VALIDATES and RECORDS the event for
the task that owns the execution lock. This node never opens the hand and never
moves the arm itself, in either mode.

SESSION TOKEN. The orchestrator creates the token and sends it with the arm
request; this node stores it and ECHOES THAT SAME VALUE in its release request.
It never generates one. Arming without a token is refused, because a release
request the orchestrator cannot identify is refused at the other end anyway.

NOT_READY OUT OF THE BOX, BY DESIGN. The tactile adapter is disabled because the
BrainCo Revo2 tactile schema is unconfirmed (the vendor ships four variants, one
with no sensor at all, and the authoritative field list is in the SDK's
main_mod.pyi, absent here), and the release thresholds, stable tolerance,
direction and debounce have no defaults because they must be measured. Arming is
refused with the exact list. See handover/README.md.
"""
from __future__ import annotations

import threading

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from handover.srv import ArmMonitor, ArmMonitorResponse

from handover import tactile_adapter as ta
from handover import tactile_frame as tf
from handover.tactile_detector import IDLE, TactileDetectorConfig, TactileReleaseDetector

#: The only service this node may ever call, and only when release integration
#: is explicitly enabled. Never a hand or arm service.
RELEASE_ROUTE_SERVICE = "/orchestrator/run_auto_release"


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _optional_number(value):
    """A calibration value must be set explicitly; empty/absent stays None."""
    if value is None or value == "":
        return None
    return float(value)


def _channel_table(raw):
    """Build {name: value} from a rosparam dict, dropping empty entries."""
    out = {}
    for key, value in (raw or {}).items():
        number = _optional_number(value)
        if number is not None:
            out[str(key)] = number
    return out


class AutoReleaseMonitorNode:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame = None
        self._frame_at = None
        self._requested = False
        #: Token issued by the orchestrator at arming. Echoed, never generated.
        self._session_token = ""

        self.side = str(_param("side", "right"))
        if self.side not in ("left", "right"):
            raise ValueError("~side must be 'left' or 'right', got %r" % (self.side,))

        self.release_integration = bool(_param("release_integration", False))
        self.rate_hz = float(_param("rate", 30.0))

        tactile = dict(_param("tactile", {}) or {})
        mappings = []
        for entry in (tactile.get("channels") or []):
            mappings.append(ta.ChannelMapping(
                name=str(entry.get("name", "")),
                finger=str(entry.get("finger", "")),
                kind=str(entry.get("kind", tf.OPAQUE)),
                field=str(entry.get("field", "")),
                unit=str(entry.get("unit", "unknown")),
                period=float(entry.get("period", 0.0) or 0.0),
            ))
        self.adapter = ta.TactileAdapterConfig(
            enabled=bool(tactile.get("enabled", False)),
            topic=str(tactile.get("topic", "")),
            message_type=str(tactile.get("message_type", "")),
            stamp_field=str(tactile.get("stamp_field", "")),
            valid_field=str(tactile.get("valid_field", "")),
            expected_rate_hz=float(tactile.get("expected_rate_hz", 0.0) or 0.0),
            mappings=tuple(mappings),
        )

        calib = dict(_param("calibration", {}) or {})
        self.config = TactileDetectorConfig(
            channels=self.adapter.specs(),
            enabled_channels=tuple(calib.get("enabled_channels", []) or []),
            # Both fall back to the detector's own defaults per channel: the
            # requested 3 N threshold and the magnitude ("either") comparison.
            release_threshold=_channel_table(calib.get("release_threshold")),
            direction={str(k): str(v) for k, v in (calib.get("direction") or {}).items() if v},
            post_arm_suppress_sec=float(_param("post_arm_suppress_sec", 1.0)),
            sample_timeout_sec=float(_param("sample_timeout_sec", 0.5)),
            monitor_timeout_sec=float(_param("monitor_timeout_sec", 120.0)),
            reference_timeout_sec=float(_param("reference_timeout_sec", 10.0)),
        )
        self.detector = TactileReleaseDetector(self.config, hand=self.side)

        self.state_pub = rospy.Publisher(
            "/auto_release_monitor/state", String, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher(
            "/auto_release_monitor/ready", Bool, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher(
            "/auto_release_monitor/event", String, queue_size=10)

        self._subscribe_tactile()
        self._publish()

    # ------------------------------------------------------------------ readiness
    def blockers(self) -> list:
        return self.adapter.validate() + self.config.validate()

    @property
    def ready(self) -> bool:
        return not self.blockers()

    def _publish(self) -> None:
        if self.ready:
            text = "%s[%s]: %s" % (self.detector.state, self.side, self.detector.last_reason)
        else:
            text = "%s[%s]: NOT_READY: %s" % (IDLE, self.side, "; ".join(self.blockers()))
        self.state_pub.publish(String(data=text))
        self.ready_pub.publish(Bool(data=self.ready))

    # ------------------------------------------------------------------ tactile in
    def _subscribe_tactile(self) -> None:
        problems = self.adapter.validate()
        if problems:
            rospy.logwarn("[auto_release_monitor:%s] no tactile input: %s",
                          self.side, "; ".join(problems))
            return
        try:
            message_class = ta.resolve_message_class(self.adapter.message_type)
        except (ImportError, ValueError) as exc:
            rospy.logerr("[auto_release_monitor:%s] cannot load tactile message type %r: %s",
                         self.side, self.adapter.message_type, exc)
            return
        rospy.Subscriber(self.adapter.topic, message_class, self._on_tactile, queue_size=5)
        rospy.loginfo(
            "[auto_release_monitor:%s] tactile input %s (%s), %d channel(s): %s",
            self.side, self.adapter.topic, self.adapter.message_type,
            len(self.adapter.mappings), ", ".join(m.name for m in self.adapter.mappings))

    def _on_tactile(self, msg) -> None:
        now = rospy.Time.now().to_sec()
        try:
            frame = ta.extract_frame(msg, self.adapter, now)
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logerr_throttle(
                5.0, "[auto_release_monitor:%s] cannot read tactile frame: %s" % (self.side, exc))
            with self._lock:
                self._frame = None          # a malformed frame is a fault, not a gap
            return
        with self._lock:
            self._frame = frame
            self._frame_at = now

    # ------------------------------------------------------------------ services
    def handle_arm(self, req) -> ArmMonitorResponse:
        """Arm this hand's detector for the session the caller names."""
        side = (getattr(req, "side", "") or "").strip()
        token = (getattr(req, "session_token", "") or "").strip()

        if side and side != self.side:
            message = ("this monitor is the %s hand, not %s" % (self.side, side))
            rospy.logwarn("[auto_release_monitor:%s] arm refused: %s", self.side, message)
            return ArmMonitorResponse(success=False, message=message, session_token="")
        if not token:
            message = ("refused: no session_token. The orchestrator must send the token it "
                       "will expect echoed back; a release request it cannot identify is "
                       "refused anyway.")
            rospy.logwarn("[auto_release_monitor:%s] arm refused: %s", self.side, message)
            return ArmMonitorResponse(success=False, message=message, session_token="")

        blockers = self.blockers()
        if blockers:
            message = "NOT_READY: " + "; ".join(blockers)
            rospy.logwarn("[auto_release_monitor:%s] arm refused: %s", self.side, message)
            self._publish()
            return ArmMonitorResponse(success=False, message=message, session_token="")

        with self._lock:
            self._requested = False
            self._frame = None
            self._frame_at = None
            # Stored, never generated here. Echoed verbatim on a verified pull.
            self._session_token = token
            outcome = self.detector.arm(rospy.Time.now().to_sec())
        rospy.loginfo("[auto_release_monitor:%s] arm (token %s): %s",
                      self.side, token, outcome.reason)
        self._publish()
        return ArmMonitorResponse(
            success=outcome.ok, message=outcome.reason,
            session_token=token if outcome.ok else "")

    def handle_disarm(self, _req) -> TriggerResponse:
        with self._lock:
            outcome = self.detector.disarm(
                rospy.Time.now().to_sec(), "disarmed by request")
            self._frame = None
            self._frame_at = None
            self._requested = False
            self._session_token = ""
        rospy.logwarn("[auto_release_monitor:%s] disarm: %s", self.side, outcome.reason)
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
            if not self.detector.armed:
                return
            frame = self._frame
            at = self._frame_at
            # Offer nothing rather than something old: the detector treats a
            # missing frame as a fault, never as evidence.
            if at is None or (now - at) > self.config.sample_timeout_sec:
                frame = None
            event = self.detector.update(now, frame)

        self._publish()
        if event.fault:
            rospy.logwarn_throttle(2.0, "[auto_release_monitor:%s] %s" % (self.side, event.reason))
            self.event_pub.publish(String(data="fault|%s|%s" % (self.side, event.reason)))
            return
        # No "candidate" stage: the consecutive-frame rule decides on one frame,
        # so there is no debounce window to report progress through. The
        # "confirmed|" wire prefix is kept unchanged -- it is the published
        # interface the orchestrator and the tests consume.
        if event.trigger:
            rospy.loginfo("[auto_release_monitor:%s] %s", self.side, event.reason)
            self.event_pub.publish(String(data="confirmed|%s|%s" % (self.side, event.reason)))
            if not self._requested:
                self._requested = True
                self._request_release(event.reason)

    def _request_release(self, reason: str) -> None:
        """Ask the ORCHESTRATOR to act, echoing the session token it issued.

        Never actuates and never opens the hand.
        """
        if not self.release_integration:
            rospy.logwarn(
                "[auto_release_monitor:%s] detection-only mode: a verified pull was "
                "published but NO release was requested and no release client exists. "
                "Set release_integration:=true once thresholds are calibrated.", self.side)
            return
        try:
            # Imported lazily so this node starts before orchestrator is built,
            # and is never imported at all in detection-only mode.
            from orchestrator.srv import RunRoute
        except ImportError as exc:
            rospy.logerr(
                "[auto_release_monitor:%s] verified pull but orchestrator messages are "
                "unavailable (%s); NOTHING was actuated.", self.side, exc)
            return
        try:
            rospy.wait_for_service(RELEASE_ROUTE_SERVICE, timeout=5.0)
            call = rospy.ServiceProxy(RELEASE_ROUTE_SERVICE, RunRoute)
            response = call(route="auto_release", target=self.side, start_step="",
                            dry_run=False, event_token=self._session_token)
            if response.success:
                rospy.loginfo("[auto_release_monitor:%s] %s accepted: %s",
                              self.side, RELEASE_ROUTE_SERVICE, response.message)
            else:
                rospy.logerr("[auto_release_monitor:%s] %s refused: %s",
                             self.side, RELEASE_ROUTE_SERVICE, response.message)
        except rospy.ROSException as exc:
            rospy.logerr("[auto_release_monitor:%s] %s unavailable (%s); nothing was actuated",
                         self.side, RELEASE_ROUTE_SERVICE, exc)
        except rospy.ServiceException as exc:
            rospy.logerr("[auto_release_monitor:%s] %s failed: %s",
                         self.side, RELEASE_ROUTE_SERVICE, exc)


def main() -> None:
    rospy.init_node("auto_release_monitor")
    node = AutoReleaseMonitorNode()

    rospy.Service("/auto_release_monitor/arm", ArmMonitor, node.handle_arm)
    rospy.Service("/auto_release_monitor/disarm", Trigger, node.handle_disarm)
    rospy.Service("/auto_release_monitor/%s/arm" % node.side, ArmMonitor, node.handle_arm)
    rospy.Service("/auto_release_monitor/%s/disarm" % node.side, Trigger, node.handle_disarm)

    mode = "RELEASE INTEGRATION ENABLED" if node.release_integration else "detection-only"
    if node.ready:
        rospy.loginfo("[auto_release_monitor:%s] ready (%s)", node.side, mode)
    else:
        rospy.logwarn("[auto_release_monitor:%s] NOT_READY (%s), arming will be refused: %s",
                      node.side, mode, "; ".join(node.blockers()))
    node.spin()


if __name__ == "__main__":
    main()
