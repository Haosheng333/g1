"""Detection-only mode must construct NO release client, even on a real trigger.

The node is driven with SYNTHETIC frames through stubbed rospy. This is not
hardware validation. The stub `rospy.ServiceProxy` raises, so if the node ever
tried to build a release client these tests would fail loudly.
"""
import os
import sys
import contextlib
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

PROXIES_BUILT = []


def _install_stubs():
    clock = {"t": 1000.0}
    params = {}

    rospy = types.ModuleType("rospy")
    rospy.ROSException = type("ROSException", (Exception,), {})
    rospy.ServiceException = type("ServiceException", (Exception,), {})
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    rospy.Service = lambda *a, **k: None
    rospy.Subscriber = lambda *a, **k: None
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.get_param = lambda name, default=None: params.get(name, default)
    rospy.wait_for_service = lambda *a, **k: None

    published = []
    def publisher(topic, *a, **k):
        return types.SimpleNamespace(
            publish=lambda msg=None, **kw: published.append((topic, msg)))
    rospy.Publisher = publisher

    def proxy(*a, **k):
        PROXIES_BUILT.append(a[0] if a else "?")
        raise AssertionError(
            "detection-only mode must not construct a release client (%r)" % (a,))
    rospy.ServiceProxy = proxy

    rospy.Time = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_sec=lambda: clock["t"]))
    for lvl in ("loginfo", "logwarn", "logerr", "logfatal", "logwarn_throttle",
                "logerr_throttle", "loginfo_throttle"):
        setattr(rospy, lvl, lambda *a, **k: None)
    rospy._clock = clock
    rospy._params = params
    rospy._published = published
    sys.modules["rospy"] = rospy

    std_msgs = types.ModuleType("std_msgs")
    msg = types.ModuleType("std_msgs.msg")
    msg.Bool = lambda data=False: types.SimpleNamespace(data=data)
    msg.String = lambda data="": types.SimpleNamespace(data=data)
    std_msgs.msg = msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = msg

    std_srvs = types.ModuleType("std_srvs")
    srv = types.ModuleType("std_srvs.srv")
    srv.Trigger = object
    srv.TriggerResponse = lambda success=False, message="": types.SimpleNamespace(
        success=success, message=message)
    std_srvs.srv = srv
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = srv

    hsrv = types.ModuleType("handover.srv")
    hsrv.ArmMonitor = object
    hsrv.ArmMonitorResponse = lambda success=False, message="", session_token="": \
        types.SimpleNamespace(success=success, message=message, session_token=session_token)
    sys.modules["handover.srv"] = hsrv
    return rospy


ROSPY = _install_stubs()

import auto_release_monitor as arm_node  # noqa: E402

from handover import tactile_frame as tf  # noqa: E402

DT = 1.0 / 30.0

#: A fully calibrated parameter set. These are TEST FIXTURES, not calibration
#: values for any real hand.
CALIBRATED = {
    "~side": "right",
    "~rate": 30.0,
    "~tactile": {
        "enabled": True,
        "topic": "/hand_right/tactile",
        "message_type": "std_msgs/Float32MultiArray",
        "expected_rate_hz": 30.0,
        "channels": [
            {"name": "ix_n", "finger": "index", "kind": tf.FORCE_LIKE,
             "field": "data[0]", "unit": "counts"},
            {"name": "ix_p", "finger": "index", "kind": tf.OPAQUE,
             "field": "data[1]", "unit": "counts"},
        ],
    },
    "~calibration": {
        "enabled_channels": ["ix_n"],
        "release_threshold": {"ix_n": 10.0},
        "direction": {"ix_n": "drop"},
    },
}


def build(release_integration=False, **overrides):
    ROSPY._params.clear()
    ROSPY._params.update(CALIBRATED)
    ROSPY._params["~release_integration"] = release_integration
    ROSPY._params.update(overrides)
    ROSPY._published[:] = []
    PROXIES_BUILT[:] = []
    ROSPY._clock["t"] = 1000.0
    # Subscribing is stubbed out, so frames are injected directly below.
    return arm_node.AutoReleaseMonitorNode()


ARM_REQ = types.SimpleNamespace(side="right", session_token="testtoken123")


def drive_to_trigger(node, frames=120, value_after=0.0):
    """Arm, baseline, clear suppression, then pull. Returns published events."""
    node.handle_arm(ARM_REQ)
    # one reference frame + the post-arm suppression window, at a steady level
    for _ in range(int(1.0 / DT) + 5):
        node._frame = tf.TactileFrame(ROSPY._clock["t"], {"ix_n": 100.0, "ix_p": 5.0})
        node._frame_at = ROSPY._clock["t"]
        node._tick()
        ROSPY._clock["t"] += DT
    # the pull
    for _ in range(frames):
        node._frame = tf.TactileFrame(ROSPY._clock["t"], {"ix_n": value_after, "ix_p": 5.0})
        node._frame_at = ROSPY._clock["t"]
        node._tick()
        ROSPY._clock["t"] += DT
    return [m.data for topic, m in ROSPY._published if topic.endswith("/event")]


class DetectionOnlyIsTheDefault(unittest.TestCase):
    def test_shipped_default_is_detection_only(self):
        node = build()
        self.assertFalse(node.release_integration)

    def test_a_verified_pull_builds_no_release_client(self):
        node = build(release_integration=False)
        self.assertTrue(node.ready, "; ".join(node.blockers()))
        events = drive_to_trigger(node)
        self.assertTrue(any(e.startswith("confirmed|") for e in events),
                        "the detector should still have confirmed the pull")
        self.assertEqual(PROXIES_BUILT, [], "no service client may be constructed")

    def test_the_detector_publishes_confirmed_and_no_candidate_stage(self):
        """The consecutive-frame rule decides on one frame, so there is no
        debounce window and therefore no "candidate|" progress event. The
        "confirmed|" prefix is unchanged: it is the published interface."""
        node = build()
        events = drive_to_trigger(node)
        self.assertTrue(any(e.startswith("confirmed|") for e in events))
        self.assertFalse(any(e.startswith("candidate|") for e in events))

    def test_a_fault_is_published_and_builds_no_client(self):
        node = build()
        node.handle_arm(ARM_REQ)
        for _ in range(3):
            node._frame = tf.TactileFrame(ROSPY._clock["t"], {"ix_n": 100.0, "ix_p": 5.0})
            node._frame_at = ROSPY._clock["t"]
            node._tick()
            ROSPY._clock["t"] += DT
        node._frame = None                      # dropout
        node._frame_at = None
        node._tick()
        events = [m.data for t, m in ROSPY._published if t.endswith("/event")]
        self.assertTrue(any(e.startswith("fault|") for e in events))
        self.assertEqual(PROXIES_BUILT, [])

    def test_orchestrator_messages_are_never_imported_in_detection_only(self):
        sys.modules.pop("orchestrator.srv", None)
        node = build(release_integration=False)
        drive_to_trigger(node)
        self.assertNotIn("orchestrator.srv", sys.modules,
                         "detection-only mode must not import the release service type")

    def test_the_node_never_names_a_hand_or_arm_service(self):
        with open(os.path.join(HERE, "..", "scripts", "auto_release_monitor.py")) as f:
            code = f.read()
        for forbidden in ("/hand_left", "/hand_right", "move_arm_joints",
                          "plan_to_pose", "arm_sdk", "lowcmd", "enable_motion"):
            self.assertNotIn(forbidden, code)


class ArmingRequiresSideAndToken(unittest.TestCase):
    def test_arming_without_a_token_is_refused(self):
        node = build()
        r = node.handle_arm(types.SimpleNamespace(side="right", session_token=""))
        self.assertFalse(r.success)
        self.assertIn("no session_token", r.message)

    def test_arming_the_wrong_side_is_refused(self):
        node = build()
        r = node.handle_arm(types.SimpleNamespace(side="left", session_token="t"))
        self.assertFalse(r.success)
        self.assertIn("right hand, not left", r.message)

    def test_a_successful_arm_echoes_the_token_back(self):
        node = build()
        r = node.handle_arm(ARM_REQ)
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.session_token, "testtoken123")

    def test_disarm_clears_the_stored_token(self):
        node = build()
        node.handle_arm(ARM_REQ)
        self.assertEqual(node._session_token, "testtoken123")
        node.handle_disarm(None)
        self.assertEqual(node._session_token, "")


class UncalibratedStaysNotReady(unittest.TestCase):
    def test_the_shipped_config_shape_is_not_ready(self):
        node = build(**{
            "~tactile": {"enabled": False, "topic": "", "message_type": "", "channels": []},
            "~calibration": {},
        })
        self.assertFalse(node.ready)
        blockers = "; ".join(node.blockers())
        self.assertIn("disabled", blockers)

    def test_arming_is_refused_while_not_ready(self):
        node = build(**{
            "~tactile": {"enabled": False, "topic": "", "message_type": "", "channels": []},
            "~calibration": {},
        })
        response = node.handle_arm(ARM_REQ)
        self.assertFalse(response.success)
        self.assertIn("NOT_READY", response.message)

    def test_an_unconfirmed_schema_alone_keeps_it_not_ready(self):
        """The threshold now has a requested default, so the SCHEMA is the gate.

        With the adapter disabled no channel is declared, so there is nothing
        for the 3 N threshold to apply to and the node stays NOT_READY even
        though every detection parameter has a value.
        """
        node = build(**{"~calibration": {"enabled_channels": []},
                        "~tactile": {"enabled": False, "topic": "", "message_type": "",
                                     "channels": []}})
        self.assertFalse(node.ready)
        blockers = "; ".join(node.blockers())
        self.assertIn("no force channels declared", blockers)

    def test_an_invalid_side_is_fatal(self):
        with self.assertRaises(ValueError):
            build(**{"~side": "middle"})


class _BlockFinder:
    """sys.meta_path finder that makes `prefix` and its submodules unimportable."""

    def __init__(self, prefix: str):
        self.prefix = prefix

    def find_module(self, name, path=None):            # py2-style, still consulted
        return self if self._matches(name) else None

    def find_spec(self, name, path=None, target=None):
        if self._matches(name):
            raise ImportError("blocked by test: %s" % (name,))
        return None

    def load_module(self, name):
        raise ImportError("blocked by test: %s" % (name,))

    def _matches(self, name):
        return name == self.prefix or name.startswith(self.prefix + ".")


@contextlib.contextmanager
def blocked_import(prefix: str):
    """Make `prefix` unimportable for the duration, however it is installed."""
    saved = {k: v for k, v in sys.modules.items()
             if k == prefix or k.startswith(prefix + ".")}
    for k in saved:
        del sys.modules[k]
    finder = _BlockFinder(prefix)
    sys.meta_path.insert(0, finder)
    try:
        yield
    finally:
        try:
            sys.meta_path.remove(finder)
        except ValueError:
            pass
        sys.modules.update(saved)


class ReleaseIntegrationPathIsGuarded(unittest.TestCase):
    """With integration ON the node asks the ORCHESTRATOR; it still never actuates."""

    def setUp(self):
        pkg = types.ModuleType("orchestrator")
        srv = types.ModuleType("orchestrator.srv")
        srv.RunRoute = object
        pkg.srv = srv
        sys.modules["orchestrator"] = pkg
        sys.modules["orchestrator.srv"] = srv

    def tearDown(self):
        sys.modules.pop("orchestrator.srv", None)
        sys.modules.pop("orchestrator", None)

    def test_enabling_integration_attempts_exactly_one_orchestrator_request(self):
        node = build(release_integration=True)
        self.assertTrue(node.ready, "; ".join(node.blockers()))
        # The stub ServiceProxy raises, which _request_release does not catch;
        # assert it was reached exactly once and named the orchestrator route.
        with self.assertRaises(AssertionError):
            drive_to_trigger(node)
        self.assertEqual(PROXIES_BUILT, ["/orchestrator/run_auto_release"])

    def test_a_missing_orchestrator_message_degrades_without_actuating(self):
        """If orchestrator is not built, nothing is actuated and nothing raises."""
        # Popping sys.modules is NOT enough: on a machine where orchestrator IS
        # built, the lazy `from orchestrator.srv import RunRoute` would simply
        # re-import it from the devel space and this test would assert the
        # opposite of what it claims. Block the import outright so the
        # degradation path is exercised on every machine.
        with blocked_import("orchestrator"):
            node = build(release_integration=True)
            events = drive_to_trigger(node)             # must not raise
        self.assertTrue(any(e.startswith("confirmed|") for e in events))
        self.assertEqual(PROXIES_BUILT, [])

    def test_the_only_service_it_may_call_is_the_orchestrator_route(self):
        self.assertEqual(arm_node.RELEASE_ROUTE_SERVICE, "/orchestrator/run_auto_release")


if __name__ == "__main__":
    unittest.main()
