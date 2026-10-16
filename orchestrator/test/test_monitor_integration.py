"""Production monitor -> orchestrator connection, end to end in one process.

This does NOT inject events into HandoverState directly. It instantiates the
REAL handover.AutoReleaseMonitorNode and the REAL orchestrator node, wires a
tiny in-process service bus between them, and drives a real tactile detection
through the real detector so the monitor issues a real release request that the
real validator handles.

What is real here: the detector, the monitor's arm/disarm handlers and release
client, the session token issued by the orchestrator and echoed by the monitor,
HandoverState validation, and the orchestrator's handle_release_request.

What is faked: rospy (clock, services, publishers), and the tactile FRAMES.
ALL TACTILE DATA IS SYNTHETIC AND TEST-ONLY. No ROS master, no hardware, no
sleeping, and nothing here is hardware validation.
"""
import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
for p in (os.path.join(REPO, "orchestrator", "src"),
          os.path.join(REPO, "orchestrator", "scripts"),
          os.path.join(REPO, "handover", "src"),
          os.path.join(REPO, "handover", "scripts")):
    sys.path.insert(0, p)

CLOCK = {"t": 1000.0}
BUS = {}            # service name -> handler
PROXIES = []        # every ServiceProxy ever constructed


class _Response(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _install_stubs():
    params = {}
    published = []

    rospy = types.ModuleType("rospy")
    rospy.ROSException = type("ROSException", (Exception,), {})
    rospy.ServiceException = type("ServiceException", (Exception,), {})
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    rospy.Subscriber = lambda *a, **k: None
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.Duration = lambda s: s
    rospy.get_param = lambda name, default=None: params.get(name, default)
    rospy.wait_for_service = lambda *a, **k: None
    rospy.wait_for_message = lambda *a, **k: None
    rospy.get_master = lambda: types.SimpleNamespace(
        getPublishedTopics=lambda p: [1, 2, []])
    rospy.sleep = lambda s: CLOCK.__setitem__("t", CLOCK["t"] + float(s))

    def service(name, srv_type, handler):
        BUS[name] = handler
        return types.SimpleNamespace(shutdown=lambda *a: None)
    rospy.Service = service

    def proxy(name, srv_type):
        PROXIES.append(name)

        def call(*a, **kw):
            if name not in BUS:
                raise rospy.ROSException("no server for %s" % name)
            return BUS[name](types.SimpleNamespace(**kw))
        return call
    rospy.ServiceProxy = proxy

    rospy.Publisher = lambda topic, *a, **k: types.SimpleNamespace(
        publish=lambda msg=None, **kw: published.append((topic, msg)))
    rospy.Time = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_sec=lambda: CLOCK["t"]))
    for lvl in ("loginfo", "logwarn", "logerr", "logfatal", "logwarn_throttle",
                "logerr_throttle", "loginfo_throttle"):
        setattr(rospy, lvl, lambda *a, **k: None)
    rospy._params = params
    rospy._published = published
    sys.modules["rospy"] = rospy

    for pkg, names in (("std_msgs", ("Bool", "String", "Empty")),
                       ("sensor_msgs", ("JointState",))):
        mod = types.ModuleType(pkg)
        msg = types.ModuleType("%s.msg" % pkg)
        for n in names:
            setattr(msg, n, lambda data=None, **k: types.SimpleNamespace(data=data))
        mod.msg = msg
        sys.modules[pkg] = mod
        sys.modules["%s.msg" % pkg] = msg

    srvs = types.ModuleType("std_srvs")
    srv = types.ModuleType("std_srvs.srv")
    srv.SetBool = object
    srv.Trigger = object
    srv.TriggerResponse = lambda success=False, message="": _Response(
        success=success, message=message)
    srvs.srv = srv
    sys.modules["std_srvs"] = srvs
    sys.modules["std_srvs.srv"] = srv

    hsrv = types.ModuleType("handover.srv")
    hsrv.ArmMonitor = object
    hsrv.ArmMonitorResponse = lambda success=False, message="", session_token="": _Response(
        success=success, message=message, session_token=session_token)
    sys.modules["handover.srv"] = hsrv

    osrv = types.ModuleType("orchestrator.srv")
    osrv.RunRoute = object
    osrv.RunRouteResponse = lambda **kw: _Response(**dict(
        {"success": False, "message": "", "steps_completed": [], "steps_skipped": [],
         "blockers": [], "final_step": "", "motion_issued": False, "dry_run": False,
         "object_state": "", "transfer_verified": False}, **kw))
    sys.modules["orchestrator.srv"] = osrv
    return rospy


ROSPY = _install_stubs()

import auto_release_monitor as monitor_mod  # noqa: E402
import orchestrator_node as orch_mod  # noqa: E402

from handover import tactile_frame as tf  # noqa: E402
from orchestrator import handover_state as hs  # noqa: E402

DT = 1.0 / 30.0

#: Fully calibrated monitor parameters. TEST FIXTURES, not calibration values.
MONITOR_PARAMS = {
    "~side": "right",
    "~rate": 30.0,
    "~release_integration": True,
    "~tactile": {
        "enabled": True, "topic": "/hand_right/tactile",
        "message_type": "std_msgs/Float32MultiArray", "expected_rate_hz": 30.0,
        "channels": [{"name": "ix_n", "finger": "index", "kind": tf.FORCE_LIKE,
                      "field": "data[0]", "unit": "counts"}],
    },
    "~calibration": {
        "enabled_channels": ["ix_n"], "release_threshold": {"ix_n": 10.0},
        "direction": {"ix_n": "drop"},
    },
}


class Rig(object):
    """Both real nodes, wired to each other through the in-process bus."""

    def __init__(self, **orch_params):
        BUS.clear()
        del PROXIES[:]
        CLOCK["t"] = 1000.0
        ROSPY._params.clear()
        ROSPY._params.update(MONITOR_PARAMS)
        self.monitor = monitor_mod.AutoReleaseMonitorNode()
        BUS["/auto_release_monitor/arm"] = self.monitor.handle_arm
        BUS["/auto_release_monitor/disarm"] = self.monitor.handle_disarm

        ROSPY._params.clear()
        ROSPY._params.update({"~enable_motion": True, "~side": "right"})
        ROSPY._params.update(orch_params)
        self.orch = orch_mod.OrchestratorNode()
        BUS["/orchestrator/run_auto_release"] = self.orch.handle_release_request

        # A grasp has happened: the task holds an object.
        self.orch.handover.begin("right", "bottle", release_authorized=True)
        self.arm_proxy = ROSPY.ServiceProxy("/auto_release_monitor/arm", object)

    # --- the orchestrator's real arming step, condensed -----------------------
    def arm(self):
        """Create ONE token, arm the monitor with it, then enter AWAITING_PULL."""
        token = hs.new_session_token()
        response = self.arm_proxy(side="right", session_token=token)
        assert response.success, response.message
        assert response.session_token == token, "monitor echoed a different token"
        self.orch.handover.mark_awaiting_pull(CLOCK["t"], token)
        return token

    def arm_in_window(self):
        """Arm the monitor but do NOT enter AWAITING_PULL yet.

        Reproduces the handshake window: the monitor's arm handler has
        returned, so it can detect and send a request, but the orchestrator has
        not called mark_awaiting_pull.
        """
        token = hs.new_session_token()
        self.orch.handover.begin_arming(CLOCK["t"], token)
        response = self.arm_proxy(side="right", session_token=token)
        assert response.success, response.message
        return token

    def finish_arming(self, token):
        self.orch.handover.mark_awaiting_pull(CLOCK["t"], token)

    def disarm(self):
        BUS["/auto_release_monitor/disarm"](types.SimpleNamespace())

    # --- drive the REAL detector with synthetic frames -------------------------
    def feed(self, n, value=100.0):
        for _ in range(n):
            self.monitor._frame = tf.TactileFrame(CLOCK["t"], {"ix_n": value})
            self.monitor._frame_at = CLOCK["t"]
            self.monitor._tick()
            CLOCK["t"] += DT

    def pull(self):
        """Take a reference, clear suppression, then a real pull.

        One steady frame establishes the reference under the consecutive-frame
        rule; the rest of the steady run clears the post-arm suppression window.
        The step to 0.0 is a -100 delta, far past the 10.0 test threshold.
        """
        self.feed(int(1.0 / DT) + 5, 100.0)
        self.feed(20, 0.0)


class TheProductionConnectionWorks(unittest.TestCase):
    def test_a_real_detection_reaches_the_orchestrator_and_is_accepted(self):
        rig = Rig()
        token = rig.arm()
        self.assertEqual(rig.monitor._session_token, token,
                         "the monitor must store the orchestrator's token")
        rig.pull()
        snap = rig.orch.handover.snapshot()
        self.assertTrue(snap.pending, "the orchestrator recorded no event")
        self.assertEqual(snap.session_token, token)

    def test_the_monitor_echoed_the_orchestrators_token(self):
        rig = Rig()
        token = rig.arm()
        rig.pull()
        event = rig.orch.handover.consume_pending(CLOCK["t"])
        self.assertIsNotNone(event)
        self.assertEqual(event.event_token, token)
        self.assertEqual(event.source, "auto_release_monitor")

    def test_exactly_one_request_is_sent_for_one_pull(self):
        rig = Rig()
        rig.arm()
        rig.pull()
        rig.feed(40, 0.0)                     # keep pulling
        self.assertEqual(PROXIES.count("/orchestrator/run_auto_release"), 1)

    def test_the_accepted_event_issues_no_motion(self):
        rig = Rig()
        rig.arm()
        rig.pull()
        for name in PROXIES:
            self.assertNotIn("/hand_", name)
            self.assertNotIn("move_arm_joints", name)


class TheHandshakeWindow(unittest.TestCase):
    """A pull between arming and AWAITING_PULL must be neither acted on nor lost."""

    def test_a_pull_in_the_window_is_queued_not_rejected(self):
        rig = Rig()
        token = rig.arm_in_window()
        self.assertEqual(rig.orch.handover.phase, hs.ARMING)
        rig.pull()
        snap = rig.orch.handover.snapshot()
        self.assertTrue(snap.pending, "the only event must not be lost")
        self.assertEqual(snap.phase, hs.ARMING)

    def test_a_queued_window_event_survives_entering_awaiting_pull(self):
        rig = Rig()
        token = rig.arm_in_window()
        rig.pull()
        rig.finish_arming(token)
        self.assertTrue(rig.orch.handover.snapshot().pending,
                        "entering AWAITING_PULL must not discard the queued event")
        event = rig.orch.handover.consume_pending(CLOCK["t"])
        self.assertIsNotNone(event, "the task would otherwise wait for a pull that happened")
        self.assertEqual(event.event_token, token)

    def test_a_window_event_causes_no_release_before_the_task_consumes_it(self):
        rig = Rig()
        rig.arm_in_window()
        rig.pull()
        # Queued only: nothing opened the hand, and no hand service was called.
        for name in PROXIES:
            self.assertNotIn("/hand_", name)
        self.assertEqual(rig.orch.handover.object_state, hs.HELD)

    def test_arm_failure_drops_the_queued_event_and_clears_the_token(self):
        rig = Rig()
        token = rig.arm_in_window()
        rig.pull()
        self.assertTrue(rig.orch.handover.snapshot().pending)
        # The orchestrator's arm step would now see a failure and abandon.
        rig.orch.handover.abandon("monitor refused to arm")
        snap = rig.orch.handover.snapshot()
        self.assertFalse(snap.pending, "a queued event must not survive abandonment")
        self.assertEqual(snap.session_token, "")
        late = rig.orch.handle_release_request(
            types.SimpleNamespace(route="auto_release", target="right",
                                  start_step="", dry_run=False, event_token=token))
        self.assertFalse(late.success)

    def test_echo_mismatch_abandons_and_refuses_the_event(self):
        rig = Rig()
        token = rig.arm_in_window()
        rig.pull()
        rig.orch.handover.abandon("token echo mismatch")
        self.assertFalse(rig.orch.handover.snapshot().pending)
        self.assertEqual(rig.orch.handover.object_state, hs.HELD,
                         "the object stays held when arming fails")

    def test_cancellation_in_the_window_releases_nothing(self):
        rig = Rig()
        rig.arm_in_window()
        rig.pull()
        rig.disarm()
        rig.orch.handover.abandon("cancelled")
        self.assertFalse(rig.orch.handover.snapshot().pending)
        self.assertEqual(rig.orch.handover.object_state, hs.HELD)
        for name in PROXIES:
            self.assertNotIn("/hand_", name)

    def test_rearming_after_a_window_abandon_uses_a_fresh_token(self):
        rig = Rig()
        first = rig.arm_in_window()
        rig.pull()
        rig.orch.handover.abandon("arm failed")
        rig.disarm()
        second = rig.arm_in_window()
        self.assertNotEqual(first, second)
        stale = rig.orch.handle_release_request(
            types.SimpleNamespace(route="auto_release", target="right",
                                  start_step="", dry_run=False, event_token=first))
        self.assertFalse(stale.success)
        self.assertIn("stale", stale.message)

    def test_a_mismatched_token_in_the_window_is_refused(self):
        rig = Rig()
        rig.arm_in_window()
        bad = rig.orch.handle_release_request(
            types.SimpleNamespace(route="auto_release", target="right",
                                  start_step="", dry_run=False,
                                  event_token="ffffffffffff"))
        self.assertFalse(bad.success)
        self.assertIn("stale", bad.message)


class OldTokenAfterRearming(unittest.TestCase):
    def test_a_pull_detected_before_rearming_is_refused_afterwards(self):
        rig = Rig()
        first = rig.arm()

        # The monitor detects, but the request is delayed: capture it instead.
        captured = {}

        def hold_the_request(req):
            captured["req"] = req
            return _Response(success=True, message="held for later delivery")

        BUS["/orchestrator/run_auto_release"] = hold_the_request
        rig.pull()
        self.assertEqual(captured["req"].event_token, first)

        # The handover times out and a NEW one is armed.
        rig.orch.handover.abandon("timeout")
        rig.disarm()
        second = rig.arm()
        self.assertNotEqual(first, second)

        # Now the delayed request finally lands on the real validator.
        BUS["/orchestrator/run_auto_release"] = rig.orch.handle_release_request
        late = rig.orch.handle_release_request(captured["req"])
        self.assertFalse(late.success)
        self.assertIn("stale", late.message)
        self.assertFalse(rig.orch.handover.snapshot().pending)

    def test_the_monitor_forgets_the_token_on_disarm(self):
        rig = Rig()
        rig.arm()
        rig.disarm()
        self.assertEqual(rig.monitor._session_token, "")

    def test_rearming_replaces_the_token_everywhere(self):
        rig = Rig()
        first = rig.arm()
        rig.orch.handover.abandon("timeout")
        rig.disarm()
        second = rig.arm()
        self.assertEqual(rig.monitor._session_token, second)
        self.assertEqual(rig.orch.handover.snapshot().session_token, second)
        self.assertNotEqual(first, second)


class DuplicatesAndDisarm(unittest.TestCase):
    def test_a_second_request_with_the_same_token_is_refused(self):
        rig = Rig()
        token = rig.arm()
        rig.pull()
        again = rig.orch.handle_release_request(
            types.SimpleNamespace(route="auto_release", target="right",
                                  start_step="", dry_run=False, event_token=token))
        self.assertFalse(again.success)
        self.assertIn("duplicate", again.message)

    def test_a_request_after_the_event_was_consumed_is_refused(self):
        rig = Rig()
        token = rig.arm()
        rig.pull()
        rig.orch.handover.consume_pending(CLOCK["t"])
        again = rig.orch.handle_release_request(
            types.SimpleNamespace(route="auto_release", target="right",
                                  start_step="", dry_run=False, event_token=token))
        self.assertFalse(again.success)
        self.assertIn("duplicate", again.message)

    def test_a_request_after_disarm_is_refused(self):
        rig = Rig()
        token = rig.arm()
        rig.disarm()
        rig.orch.handover.abandon("disarmed")
        late = rig.orch.handle_release_request(
            types.SimpleNamespace(route="auto_release", target="right",
                                  start_step="", dry_run=False, event_token=token))
        self.assertFalse(late.success)
        self.assertIn("not waiting", late.message)

    def test_a_disarmed_monitor_detects_nothing(self):
        rig = Rig()
        rig.arm()
        rig.disarm()
        rig.pull()
        self.assertEqual(PROXIES.count("/orchestrator/run_auto_release"), 0)
        self.assertFalse(rig.orch.handover.snapshot().pending)


class TimeoutAndRefusals(unittest.TestCase):
    def test_a_request_after_the_awaiting_window_is_refused_as_stale(self):
        rig = Rig()
        token = rig.arm()
        CLOCK["t"] += 10_000.0                       # long past max_awaiting_sec
        late = rig.orch.handle_release_request(
            types.SimpleNamespace(route="auto_release", target="right",
                                  start_step="", dry_run=False, event_token=token))
        self.assertFalse(late.success)
        self.assertIn("stale", late.message)

    def test_a_tokenless_request_is_refused(self):
        rig = Rig()
        rig.arm()
        bare = rig.orch.handle_release_request(
            types.SimpleNamespace(route="auto_release", target="right",
                                  start_step="", dry_run=False, event_token=""))
        self.assertFalse(bare.success)
        self.assertIn("origin handover cannot be established", bare.message)

    def test_arming_is_refused_without_a_token(self):
        rig = Rig()
        response = rig.arm_proxy(side="right", session_token="")
        self.assertFalse(response.success)
        self.assertIn("no session_token", response.message)

    def test_arming_the_wrong_side_is_refused(self):
        rig = Rig()
        response = rig.arm_proxy(side="left", session_token=hs.new_session_token())
        self.assertFalse(response.success)
        self.assertIn("right hand, not left", response.message)

    def test_detection_only_mode_sends_no_request_at_all(self):
        params = dict(MONITOR_PARAMS)
        params["~release_integration"] = False
        BUS.clear()
        del PROXIES[:]
        CLOCK["t"] = 1000.0
        ROSPY._params.clear()
        ROSPY._params.update(params)
        node = monitor_mod.AutoReleaseMonitorNode()
        BUS["/auto_release_monitor/arm"] = node.handle_arm
        node.handle_arm(types.SimpleNamespace(side="right", session_token="tok123456789"))
        for _ in range(int(1.0 / DT) + 25):
            node._frame = tf.TactileFrame(CLOCK["t"], {"ix_n": 100.0})
            node._frame_at = CLOCK["t"]
            node._tick()
            CLOCK["t"] += DT
        for _ in range(20):
            node._frame = tf.TactileFrame(CLOCK["t"], {"ix_n": 0.0})
            node._frame_at = CLOCK["t"]
            node._tick()
            CLOCK["t"] += DT
        self.assertEqual(PROXIES.count("/orchestrator/run_auto_release"), 0)


if __name__ == "__main__":
    unittest.main()
