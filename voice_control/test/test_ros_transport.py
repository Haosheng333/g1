#!/usr/bin/env python3
"""REAL ROS transport test: a genuine roscore, genuine rospy.Service servers
standing in for the orchestrator (no hardware SDK and no motion backend),
and RosRouteClient calling them over actual
ROS service transport -- not sys.modules substitution.

This is NOT run by `catkin_add_nosetests` (see CMakeLists.txt): those are
pure in-process tests (test_commands.py, test_dispatch.py) or mock rospy at
the Python-import level (test_orchestrator_client.py) specifically so they
need no ROS master. This file needs one. Run it with:

    rostest g1_voice_control test_ros_transport.test

(add a matching .test launch file, or `rosrun rostest rostest` pointed at
this module, per your workspace's rostest setup) on a machine with ROS1
Noetic. Untested here -- no roscore, no catkin workspace on this machine;
verify it actually runs before relying on it.
"""

from __future__ import annotations

import unittest

try:
    import rospy
    import rostest
except ImportError as exc:  # pragma: no cover - this whole file needs real ROS
    # Skip cleanly rather than erroring, so running the offline suite
    # (`python3 -m unittest discover`) without ROS installed reports this
    # file as SKIPPED, not FAILED -- it is not broken, it genuinely needs a
    # ROS environment this machine does not have. `rostest` itself (which
    # only runs where ROS exists) is unaffected by this guard.
    raise unittest.SkipTest(f"test_ros_transport.py requires a real ROS1 environment: {exc}")


# Use the actual generated service type while hosting fake service callbacks.
# This validates serialization and request-field mapping without starting the
# real orchestrator node or any hardware backend.
from orchestrator.srv import RunRoute, RunRouteResponse  # noqa: E402


class FakeOrchestratorServer:
    """Stands in for the real orchestrator node: two services, canned
    responses, no robotics, no hardware SDK."""

    def __init__(self):
        self.received_requests = []
        self._grasp_srv = rospy.Service("/orchestrator/run_grasp", RunRoute, self._handle_grasp)
        self._deliver_srv = rospy.Service("/orchestrator/run_deliver", RunRoute, self._handle_deliver)

    def _handle_grasp(self, req):
        self.received_requests.append(("grasp", req))
        return RunRouteResponse(
            success=False,
            message="NOT_READY: 1 blocker(s); no step executed",
            steps_completed=[],
            steps_skipped=[],
            blockers=["perception_extrinsics (perception): unverified"],
            final_step="",
            motion_issued=False,
            dry_run=req.dry_run,
            object_state="NONE",
            transfer_verified=False,
        )

    def _handle_deliver(self, req):
        self.received_requests.append(("deliver", req))
        return RunRouteResponse(
            success=True,
            message="route 'deliver' completed (14 steps)",
            steps_completed=["require_held_object"],
            steps_skipped=[],
            blockers=[],
            final_step="execute_retract_home",
            motion_issued=not req.dry_run,
            dry_run=req.dry_run,
            object_state="NONE",
            transfer_verified=False,
        )

    def shutdown(self):
        self._grasp_srv.shutdown()
        self._deliver_srv.shutdown()


class TestRosTransport(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_ros_transport", anonymous=True)
        cls.fake_server = FakeOrchestratorServer()

    @classmethod
    def tearDownClass(cls):
        cls.fake_server.shutdown()

    def setUp(self):
        self.fake_server.received_requests.clear()

    def test_grasp_call_travels_over_real_transport_and_reports_not_ready(self):
        from g1_voice_control.orchestrator_client import RosRouteClient

        client = RosRouteClient(service_wait_timeout_s=5.0)
        result = client.call_route("grasp", target="bottle", dry_run=False)

        self.assertFalse(result.success)
        self.assertEqual(result.blockers, ("perception_extrinsics (perception): unverified",))
        self.assertEqual(len(self.fake_server.received_requests), 1)
        route_name, req = self.fake_server.received_requests[0]
        self.assertEqual(req.target, "bottle")
        self.assertEqual(req.dry_run, False)

    def test_deliver_call_round_trips_dry_run_flag(self):
        from g1_voice_control.orchestrator_client import RosRouteClient

        client = RosRouteClient(service_wait_timeout_s=5.0)
        result = client.call_route("deliver", target="sofa", dry_run=True)

        self.assertTrue(result.success)
        self.assertTrue(result.dry_run)
        self.assertFalse(result.motion_issued, "a dry-run request must not report motion_issued")


if __name__ == "__main__":
    rostest.rosrun("g1_voice_control", "test_ros_transport", TestRosTransport)
