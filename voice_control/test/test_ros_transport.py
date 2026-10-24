#!/usr/bin/env python3
"""REAL ROS transport test: a genuine roscore, a fake orchestrator that
subscribes to /voice/command and answers on /orchestrator/command_result with
the real generated message types, and TopicCommandClient publishing to it.

Not run by `catkin_add_nosetests` (those tests need no master). Run with:

    rostest g1_voice_control test_ros_transport.test
"""

from __future__ import annotations

import unittest

try:
    import rospy
    import rostest
except ImportError as exc:  # pragma: no cover - this whole file needs real ROS
    raise unittest.SkipTest(f"test_ros_transport.py requires a real ROS1 environment: {exc}")

from g1_voice_control.msg import TaskCommand, TaskResult  # noqa: E402


class FakeOrchestrator:
    """Stands in for the real orchestrator node: subscribes, answers, no robotics."""

    def __init__(self):
        self.received = []
        self._pub = rospy.Publisher("/orchestrator/command_result", TaskResult, queue_size=10)
        self._sub = rospy.Subscriber("/voice/command", TaskCommand, self._on_command)

    def _on_command(self, msg):
        self.received.append(msg)
        self._pub.publish(TaskResult(
            seq=msg.seq, command_id=msg.command_id, accepted=True,
            success=(msg.command_id == "fetch_deliver"),
            message="DRY RUN: route %r completed" % msg.command_id if msg.command_id == "fetch_deliver"
            else "NOT_READY: 1 blocker(s); no step executed",
            blockers=[] if msg.command_id == "fetch_deliver" else ["perception_extrinsics (perception): unverified"],
            dry_run=True, motion_issued=False, final_step=""))


class TestTopicTransport(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_voice_topic_transport")
        cls.fake = FakeOrchestrator()
        from g1_voice_control.orchestrator_client import TopicCommandClient
        cls.client = TopicCommandClient(result_timeout_s=10.0, subscriber_wait_s=10.0)

    def test_fetch_deliver_round_trip(self):
        result = self.client.call_route("fetch_deliver", target="table_01", dry_run=False,
                                        destination="chair_01", label="bottle")
        self.assertTrue(result.success, result.message)
        self.assertTrue(result.dry_run)
        cmd = self.fake.received[-1]
        self.assertEqual((cmd.command_id, cmd.target, cmd.destination, cmd.label),
                         ("fetch_deliver", "table_01", "chair_01", "bottle"))

    def test_blockers_cross_the_wire(self):
        result = self.client.call_route("grasp", target="bottle", dry_run=False)
        self.assertFalse(result.success)
        self.assertEqual(len(result.blockers), 1)
        self.assertIn("perception_extrinsics", result.blockers[0])


if __name__ == "__main__":
    rostest.rosrun("g1_voice_control", "test_ros_transport", TestTopicTransport)
