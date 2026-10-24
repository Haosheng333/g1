"""TopicCommandClient's round trip WITHOUT a real ROS master: rospy and the
generated message module are replaced with minimal fakes in sys.modules before
import, so the actual call_route() code path runs offline. Transport over a real
master is test_ros_transport.py's job.
"""

from __future__ import annotations

import sys
import threading
import types
import unittest


def _install_fake_ros_modules():
    fake_rospy = types.ModuleType("rospy")
    clock = {"t": 0.0}
    fake_rospy.get_time = lambda: clock["t"]
    fake_rospy.sleep = lambda s: clock.__setitem__("t", clock["t"] + float(s))
    fake_rospy.is_shutdown = lambda: False

    class FakePublisher:
        connections = 1
        published: list = []

        def __init__(self, topic, msg_type, queue_size=1):
            self.topic, self.msg_type = topic, msg_type

        def get_num_connections(self):
            return FakePublisher.connections

        def publish(self, msg):
            FakePublisher.published.append(msg)

    class FakeSubscriber:
        instances: list = []

        def __init__(self, topic, msg_type, callback, queue_size=1):
            self.topic, self.callback = topic, callback
            FakeSubscriber.instances.append(self)

    fake_rospy.Publisher = FakePublisher
    fake_rospy.Subscriber = FakeSubscriber

    class Msg:
        def __init__(self, **kw):
            for k, v in kw.items():
                setattr(self, k, v)

    class TaskCommand(Msg):
        pass

    class TaskResult(Msg):
        def __init__(self, seq=0, command_id="", accepted=True, success=False, message="",
                     blockers=(), dry_run=True, motion_issued=False, final_step=""):
            super().__init__(seq=seq, command_id=command_id, accepted=accepted, success=success,
                             message=message, blockers=list(blockers), dry_run=dry_run,
                             motion_issued=motion_issued, final_step=final_step)

    fake_msg = types.ModuleType("g1_voice_control.msg")
    fake_msg.TaskCommand, fake_msg.TaskResult = TaskCommand, TaskResult

    sys.modules["rospy"] = fake_rospy
    sys.modules["g1_voice_control.msg"] = fake_msg
    return fake_rospy, FakePublisher, FakeSubscriber, TaskResult


class TestTopicCommandClient(unittest.TestCase):
    def setUp(self):
        self._saved = {n: sys.modules.get(n) for n in
                       ("rospy", "g1_voice_control.msg", "g1_voice_control.orchestrator_client")}
        self.rospy, self.Pub, self.Sub, self.TaskResult = _install_fake_ros_modules()
        self.Pub.published, self.Pub.connections, self.Sub.instances = [], 1, []
        sys.modules.pop("g1_voice_control.orchestrator_client", None)
        import g1_voice_control.orchestrator_client as oc
        self.oc = oc

    def tearDown(self):
        for name, mod in self._saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod

    def _answer(self, client, **fields):
        """Deliver a TaskResult for the last published command, from another thread."""
        def deliver():
            while not self.Pub.published:
                pass
            cmd = self.Pub.published[-1]
            result = self.TaskResult(seq=cmd.seq, command_id=cmd.command_id, **fields)
            self.Sub.instances[0].callback(result)
        threading.Thread(target=deliver, daemon=True).start()

    def test_the_contract_topics(self):
        client = self.oc.TopicCommandClient()
        self.assertEqual(client.command_topic, "/voice/command")
        self.assertEqual(client.result_topic, "/orchestrator/command_result")
        self.assertEqual(self.Sub.instances[0].topic, "/orchestrator/command_result")

    def test_fetch_deliver_publishes_every_slot_and_returns_the_matching_result(self):
        client = self.oc.TopicCommandClient(result_timeout_s=2.0)
        self._answer(client, success=True, message="DRY RUN: route 'fetch_deliver' completed",
                     dry_run=True)
        result = client.call_route("fetch_deliver", target="table_01", dry_run=False,
                                   destination="chair_01", label="bottle", summary="I will ...",
                                   transcript="go to the desk ... over")
        cmd = self.Pub.published[-1]
        self.assertEqual(cmd.seq, 1)
        self.assertEqual((cmd.command_id, cmd.target, cmd.destination, cmd.label),
                         ("fetch_deliver", "table_01", "chair_01", "bottle"))
        self.assertEqual(cmd.summary, "I will ...")
        self.assertTrue(result.success)
        self.assertTrue(result.dry_run)
        self.assertIn("completed", result.message)

    def test_blockers_and_failure_fields_pass_through(self):
        client = self.oc.TopicCommandClient(result_timeout_s=2.0)
        self._answer(client, success=False, message="NOT_READY: refusing to move; 2 blocker(s)",
                     blockers=["a (x): one", "b (y): two"], dry_run=False)
        result = client.call_route("fetch_deliver", target="table_01", dry_run=False,
                                   destination="chair_01", label="bottle")
        self.assertFalse(result.success)
        self.assertEqual(result.blockers, ("a (x): one", "b (y): two"))
        self.assertFalse(result.dry_run)

    def test_a_result_for_another_seq_is_ignored(self):
        client = self.oc.TopicCommandClient(result_timeout_s=0.5)
        self.Sub.instances[0].callback(self.TaskResult(seq=99, success=True, message="stale"))
        result = client.call_route("grasp", target="bottle", dry_run=False)
        self.assertFalse(result.success)
        self.assertIn("no result", result.message)

    def test_sequence_numbers_increase(self):
        client = self.oc.TopicCommandClient(result_timeout_s=0.2)
        client.call_route("grasp", target="bottle", dry_run=False)
        client.call_route("deliver", target="sofa", dry_run=False)
        self.assertEqual([m.seq for m in self.Pub.published], [1, 2])

    def test_no_subscriber_is_a_failure_not_a_lost_command(self):
        self.Pub.connections = 0
        client = self.oc.TopicCommandClient(subscriber_wait_s=1.0)
        result = client.call_route("fetch_deliver", target="table_01", dry_run=False,
                                   destination="chair_01", label="bottle")
        self.assertFalse(result.success)
        self.assertIn("nobody subscribes", result.message)
        self.assertEqual(self.Pub.published, [])

    def test_unmapped_command_is_refused_before_publishing(self):
        client = self.oc.TopicCommandClient()
        with self.assertRaises(ValueError):
            client.call_route("fly", target="moon", dry_run=False)
        self.assertEqual(self.Pub.published, [])

    def test_status_reader_reports_latest_or_absence(self):
        fake_std = types.ModuleType("std_msgs.msg")
        fake_std.String = object
        saved = sys.modules.get("std_msgs.msg")
        sys.modules["std_msgs.msg"] = fake_std
        try:
            reader = self.oc.StatusReader()
            self.assertIn("no status received", reader.latest())
            self.Sub.instances[-1].callback(types.SimpleNamespace(data="idle; MOTION ENABLED"))
            self.assertEqual(reader.latest(), "idle; MOTION ENABLED")
        finally:
            if saved is None:
                sys.modules.pop("std_msgs.msg", None)
            else:
                sys.modules["std_msgs.msg"] = saved


if __name__ == "__main__":
    unittest.main()
