"""Tests orchestrator_client's request-field mapping WITHOUT a real ROS
master: rospy and orchestrator.srv are replaced with minimal fakes in
sys.modules before import, so this exercises the actual call_route() code
path (not a reimplementation of it) while staying offline. This is the kind
of in-process substitution the spec calls a fake-service test, distinct from
test_ros_transport.py's real rostest-over-loopback transport test.
"""

from __future__ import annotations

import sys
import types
import unittest
from unittest import mock


def _install_fake_ros_modules():
    """Registers fake `rospy` and `orchestrator.srv` modules so
    orchestrator_client's module-level `import rospy` and its lazy
    `from orchestrator.srv import RunRoute` both succeed."""

    fake_rospy = types.ModuleType("rospy")
    fake_rospy.wait_for_service = mock.Mock()
    fake_rospy.get_time = lambda: 0.0

    class FakeServiceProxy:
        """Records the keyword arguments of the last call and returns a
        pre-set response object built by the test."""

        _next_response = None  # set per-test via module-level hook below

        def __init__(self, service_name, srv_type):
            self.service_name = service_name
            self.srv_type = srv_type
            self.calls = []

        def __call__(self, **kwargs):
            self.calls.append(kwargs)
            if FakeServiceProxy._next_response is None:
                raise RuntimeError("test did not set FakeServiceProxy._next_response")
            return FakeServiceProxy._next_response

    fake_rospy.ServiceProxy = FakeServiceProxy

    class FakeSubscriber:
        def __init__(self, topic, msg_type, callback):
            self.topic = topic
            self.callback = callback

    fake_rospy.Subscriber = FakeSubscriber

    fake_orchestrator = types.ModuleType("orchestrator")
    fake_orchestrator_srv = types.ModuleType("orchestrator.srv")

    class FakeRunRouteResponse:
        def __init__(self, success, message, blockers=None, dry_run=True, motion_issued=False):
            self.success = success
            self.message = message
            self.blockers = blockers or []
            self.dry_run = dry_run
            self.motion_issued = motion_issued

    class FakeRunRoute:
        Response = FakeRunRouteResponse

    fake_orchestrator_srv.RunRoute = FakeRunRoute

    sys.modules["rospy"] = fake_rospy
    sys.modules["orchestrator"] = fake_orchestrator
    sys.modules["orchestrator.srv"] = fake_orchestrator_srv
    return fake_rospy, FakeServiceProxy, FakeRunRouteResponse


class TestRosRouteClient(unittest.TestCase):
    def setUp(self):
        self._saved_modules = {
            name: sys.modules.get(name) for name in ("rospy", "orchestrator", "orchestrator.srv", "g1_voice_control.orchestrator_client")
        }
        self.fake_rospy, self.FakeServiceProxy, self.FakeResponse = _install_fake_ros_modules()
        # Force a fresh import against the fakes just installed.
        sys.modules.pop("g1_voice_control.orchestrator_client", None)
        import g1_voice_control.orchestrator_client as oc

        self.oc = oc

    def tearDown(self):
        for name, mod in self._saved_modules.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod

    def test_grasp_maps_to_run_grasp_service_with_correct_fields(self):
        self.FakeServiceProxy._next_response = self.FakeResponse(
            success=True, message="route 'grasp' completed (0 steps)", dry_run=False
        )
        client = self.oc.RosRouteClient()
        result = client.call_route("grasp", target="bottle", dry_run=False)

        proxy = client._proxies["grasp"]
        self.assertEqual(proxy.service_name, "/orchestrator/run_grasp")
        self.assertEqual(
            proxy.calls[-1],
            {"route": "grasp", "target": "bottle", "destination": "", "label": "", "start_step": "",
             "dry_run": False, "event_token": ""},
        )
        self.assertTrue(result.success)

    def test_deliver_maps_to_run_deliver_service(self):
        self.FakeServiceProxy._next_response = self.FakeResponse(success=True, message="ok")
        client = self.oc.RosRouteClient()
        client.call_route("deliver", target="sofa", dry_run=True)

        proxy = client._proxies["deliver"]
        self.assertEqual(proxy.service_name, "/orchestrator/run_deliver")
        self.assertEqual(proxy.calls[-1]["target"], "sofa")
        self.assertEqual(proxy.calls[-1]["dry_run"], True)

    def test_fetch_deliver_maps_to_run_fetch_deliver_with_both_waypoints(self):
        self.FakeServiceProxy._next_response = self.FakeResponse(success=True, message="ok")
        client = self.oc.RosRouteClient()
        client.call_route("fetch_deliver", target="table_01", dry_run=False,
                          destination="chair_01", label="bottle")

        proxy = client._proxies["fetch_deliver"]
        self.assertEqual(proxy.service_name, "/orchestrator/run_fetch_deliver")
        self.assertEqual(
            proxy.calls[-1],
            {"route": "fetch_deliver", "target": "table_01", "destination": "chair_01",
             "label": "bottle", "start_step": "", "dry_run": False, "event_token": ""},
        )

    def test_blockers_and_failure_fields_pass_through(self):
        self.FakeServiceProxy._next_response = self.FakeResponse(
            success=False,
            message="NOT_READY: 1 blocker(s); no step executed",
            blockers=["perception_extrinsics (perception): unverified"],
        )
        client = self.oc.RosRouteClient()
        result = client.call_route("grasp", target="bottle", dry_run=False)

        self.assertFalse(result.success)
        self.assertEqual(result.blockers, ("perception_extrinsics (perception): unverified",))

    def test_unmapped_command_raises_rather_than_guessing_a_service(self):
        client = self.oc.RosRouteClient()
        with self.assertRaises(ValueError):
            client.call_route("cancel", target="", dry_run=True)

    def test_service_proxy_created_lazily_not_at_construction(self):
        client = self.oc.RosRouteClient()
        self.assertEqual(client._proxies, {}, "no service proxy should exist before the first call")


if __name__ == "__main__":
    unittest.main()
