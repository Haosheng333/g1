import threading
import unittest

from g1_voice_control.dispatch import (
    Dispatcher,
    RecordingDryRunClient,
    RouteCallResult,
    State,
)


class FakeClock:
    def __init__(self, start: float = 0.0):
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        self.t += seconds


class FakeRouteClient:
    """Records every call and returns a pre-set result, so tests can check
    exactly what request fields reached the client and nothing else did."""

    def __init__(self, result: RouteCallResult = None):
        self.calls = []
        self._result = result or RouteCallResult(success=True, message="ok")
        self.raise_exc = None

    def call_route(self, command_id, target, dry_run):
        self.calls.append({"command_id": command_id, "target": target, "dry_run": dry_run})
        if self.raise_exc is not None:
            raise self.raise_exc
        return self._result


def make_dispatcher(client=None, clock=None, window=8.0, targets=None):
    targets = targets or {"grasp": "bottle", "deliver": "sofa"}
    clock = clock or FakeClock()
    client = client or FakeRouteClient()
    return (
        Dispatcher(
            route_client=client,
            clock=clock,
            confirmation_window_sec=window,
            target_resolver=lambda cmd: targets.get(cmd),
        ),
        client,
        clock,
    )


def confirm_and_execute(dispatcher):
    prepared = dispatcher.handle_command("confirm")
    if prepared.dispatch_action is None:
        return prepared
    return dispatcher.execute_action(prepared.dispatch_action)


class TestConfirmFlow(unittest.TestCase):
    def test_grasp_requires_confirmation_before_dispatch(self):
        d, client, clock = make_dispatcher()
        event = d.handle_command("grasp")
        self.assertEqual(event.kind, "awaiting_confirmation")
        self.assertEqual(d.state, State.PENDING_CONFIRM)
        self.assertEqual(client.calls, [], "no call should happen before confirmation")

    def test_confirm_dispatches_with_correct_fields(self):
        d, client, clock = make_dispatcher()
        d.handle_command("grasp")
        event = d.handle_command("confirm")
        self.assertEqual(
            event.kind,
            "dispatch_ready",
            "confirmation must finish its state transition before the blocking service call starts",
        )
        self.assertEqual(client.calls, [])
        event = d.execute_action(event.dispatch_action)
        self.assertEqual(event.kind, "dispatched")
        self.assertEqual(client.calls, [{"command_id": "grasp", "target": "bottle", "dry_run": False}])
        self.assertEqual(d.state, State.IDLE)

    def test_confirm_with_nothing_pending_is_ignored(self):
        d, client, clock = make_dispatcher()
        event = d.handle_command("confirm")
        self.assertEqual(event.kind, "ignored")
        self.assertEqual(client.calls, [])


class TestExpiry(unittest.TestCase):
    def test_pending_expires_after_window(self):
        d, client, clock = make_dispatcher(window=5.0)
        d.handle_command("grasp")
        clock.advance(5.1)
        event = d.handle_command("confirm")
        self.assertEqual(event.kind, "ignored")
        self.assertEqual(client.calls, [], "an expired pending command must not dispatch")
        self.assertIsNone(d.pending)

    def test_confirm_just_inside_window_still_dispatches(self):
        d, client, clock = make_dispatcher(window=5.0)
        d.handle_command("grasp")
        clock.advance(4.9)
        event = confirm_and_execute(d)
        self.assertEqual(event.kind, "dispatched")
        self.assertEqual(len(client.calls), 1)

    def test_poll_expiry_clears_pending_without_another_command(self):
        d, client, clock = make_dispatcher(window=5.0)
        d.handle_command("grasp")
        clock.advance(5.1)
        event = d.poll_expiry()
        self.assertIsNotNone(event)
        self.assertEqual(event.kind, "expired")
        self.assertIsNone(d.pending)
        self.assertEqual(d.state, State.IDLE)


class TestDuplicateAndReplace(unittest.TestCase):
    def test_second_task_start_before_confirm_replaces_pending_not_stacks(self):
        d, client, clock = make_dispatcher()
        d.handle_command("grasp")
        d.handle_command("deliver")
        self.assertEqual(d.pending.command_id, "deliver")
        confirm_and_execute(d)
        self.assertEqual(client.calls, [{"command_id": "deliver", "target": "sofa", "dry_run": False}])

    def test_repeated_identical_utterance_does_not_double_dispatch(self):
        d, client, clock = make_dispatcher()
        d.handle_command("grasp")
        d.handle_command("grasp")  # heard twice (e.g. ASR repeats a final result)
        confirm_and_execute(d)
        self.assertEqual(len(client.calls), 1)


class TestBusyRejection(unittest.TestCase):
    def test_task_start_rejected_while_busy(self):
        d, client, clock = make_dispatcher()

        # White-box check of the documented state invariant. The separate
        # blocking-client test below exercises the real concurrent path.
        d.state = State.BUSY
        d.last_route_command = "deliver"
        event = d.handle_command("grasp")
        self.assertEqual(event.kind, "rejected")
        self.assertEqual(client.calls, [])

    def test_confirm_ignored_while_busy(self):
        d, client, clock = make_dispatcher()
        d.state = State.BUSY
        event = d.handle_command("confirm")
        self.assertEqual(event.kind, "ignored")
        self.assertEqual(client.calls, [])

    def test_new_task_is_rejected_while_service_call_is_blocked(self):
        entered = threading.Event()
        release = threading.Event()

        class BlockingClient(FakeRouteClient):
            def call_route(self, command_id, target, dry_run):
                self.calls.append({"command_id": command_id, "target": target, "dry_run": dry_run})
                entered.set()
                release.wait(1.0)
                return RouteCallResult(success=True, message="ok")

        d, client, clock = make_dispatcher(client=BlockingClient())
        d.handle_command("grasp")
        prepared = d.handle_command("confirm")
        worker = threading.Thread(target=d.execute_action, args=(prepared.dispatch_action,))
        worker.start()
        self.assertTrue(entered.wait(0.5))

        event = d.handle_command("deliver")
        self.assertEqual(event.kind, "rejected")
        self.assertIn("busy", event.detail)

        release.set()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())


class TestMissingTarget(unittest.TestCase):
    def test_missing_target_is_reported_not_invented(self):
        d, client, clock = make_dispatcher(targets={"grasp": None, "deliver": "sofa"})
        event = d.handle_command("grasp")
        self.assertEqual(event.kind, "rejected")
        self.assertIn("no target configured", event.detail)
        self.assertEqual(client.calls, [])
        self.assertEqual(d.state, State.IDLE, "a missing target must not leave a pending command behind")


class TestCancel(unittest.TestCase):
    def test_cancel_clears_pending_and_reports_locally_handled(self):
        d, client, clock = make_dispatcher()
        d.handle_command("grasp")
        event = d.handle_command("cancel")
        self.assertEqual(event.kind, "cancelled")
        self.assertIsNone(d.pending)
        self.assertEqual(d.state, State.IDLE)
        self.assertEqual(client.calls, [], "cancel must never itself call the orchestrator")

    def test_cancel_with_nothing_pending_reports_nothing_pending(self):
        d, client, clock = make_dispatcher()
        event = d.handle_command("cancel")
        self.assertIn("nothing was pending", event.detail)

    def test_cancel_while_busy_reports_the_real_gap_not_success(self):
        d, client, clock = make_dispatcher()
        d.state = State.BUSY
        event = d.handle_command("cancel")
        self.assertEqual(event.kind, "unavailable")
        self.assertIn("no cancellation service", event.detail)
        self.assertIn("cannot be stopped", event.detail)


class TestFaultHandling(unittest.TestCase):
    def test_audio_fault_clears_a_pending_confirmation(self):
        d, client, clock = make_dispatcher()
        d.handle_command("grasp")
        event = d.handle_fault("input overflow")
        self.assertEqual(event.kind, "fault")
        self.assertIsNone(d.pending)
        self.assertEqual(d.state, State.IDLE)
        self.assertEqual(client.calls, [])


class TestServiceFailureNoAutoRetry(unittest.TestCase):
    def test_failed_call_is_reported_once_no_retry(self):
        client = FakeRouteClient(result=RouteCallResult(success=False, message="TIMEOUT"))
        d, client, clock = make_dispatcher(client=client)
        d.handle_command("grasp")
        event = confirm_and_execute(d)
        self.assertEqual(event.kind, "dispatched")
        self.assertFalse(event.route_result.success)
        self.assertEqual(len(client.calls), 1, "an uncertain/failed call must not be auto-retried")
        self.assertEqual(d.state, State.IDLE)

    def test_raising_client_is_reported_as_failure_not_a_crash(self):
        client = FakeRouteClient()
        client.raise_exc = RuntimeError("rospy service call failed")
        d, client, clock = make_dispatcher(client=client)
        d.handle_command("deliver")
        event = confirm_and_execute(d)
        self.assertEqual(event.kind, "dispatched")
        self.assertFalse(event.route_result.success)
        self.assertIn("RuntimeError", event.route_result.message)


class TestDryRunClientMakesNoRealCalls(unittest.TestCase):
    def test_recording_client_records_and_reports_dry_run(self):
        client = RecordingDryRunClient()
        result = client.call_route("grasp", "bottle", dry_run=True)
        self.assertTrue(result.dry_run)
        self.assertTrue(result.success)
        self.assertEqual(client.calls, [{"command_id": "grasp", "target": "bottle", "dry_run": True}])

    def test_dispatcher_with_recording_client_never_records_real_dispatch(self):
        client = RecordingDryRunClient()
        d, client, clock = make_dispatcher(client=client)
        d.handle_command("grasp")
        result = confirm_and_execute(d)
        self.assertTrue(result.route_result.dry_run)
        self.assertEqual(client.calls, [{"command_id": "grasp", "target": "bottle", "dry_run": True}])


if __name__ == "__main__":
    unittest.main()
