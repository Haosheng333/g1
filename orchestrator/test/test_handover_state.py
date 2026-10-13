"""Release-request validation and ownership: no deadlock, no lock bypass.

SYNTHETIC inputs only; no ROS, no hardware. This is not hardware validation.
"""
import ast
import io
import os
import sys
import threading
import time
import tokenize
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from orchestrator import handover_state as hs  # noqa: E402
from orchestrator.execution_lock import Busy, ExecutionLock  # noqa: E402

NODE = os.path.join(HERE, "..", "scripts", "orchestrator_node.py")


def holding_and_awaiting(authorized=True, now=10.0, side="right"):
    state = hs.HandoverState()
    state.begin(side, "bottle", release_authorized=authorized)
    state.mark_awaiting_pull(now)
    return state


class PhasesAndBookkeeping(unittest.TestCase):
    def test_a_fresh_state_holds_nothing(self):
        state = hs.HandoverState()
        snap = state.snapshot()
        self.assertEqual(snap.phase, hs.IDLE)
        self.assertFalse(snap.holding)

    def test_begin_records_the_held_object_and_moves(self):
        state = hs.HandoverState()
        state.begin("left", "bottle", release_authorized=True)
        snap = state.snapshot()
        self.assertEqual(snap.phase, hs.MOVING_TO_POSE)
        self.assertEqual(snap.side, "left")
        self.assertEqual(snap.label, "bottle")
        self.assertTrue(snap.holding)

    def test_begin_rejects_a_bad_side(self):
        with self.assertRaises(ValueError):
            hs.HandoverState().begin("middle", "bottle", True)

    def test_awaiting_requires_a_held_object(self):
        with self.assertRaises(RuntimeError):
            hs.HandoverState().mark_awaiting_pull(0.0)

    def test_each_handover_gets_a_new_session(self):
        state = hs.HandoverState()
        state.begin("right", "a", True)
        first = state.snapshot().session
        state.begin("right", "b", True)
        self.assertGreater(state.snapshot().session, first)

    def test_released_clears_the_held_object(self):
        state = holding_and_awaiting()
        state.mark_released()
        snap = state.snapshot()
        self.assertFalse(snap.holding)
        self.assertEqual(snap.phase, hs.IDLE)

    def test_phases_are_the_documented_set(self):
        self.assertEqual(hs.PHASES, ("IDLE", "MOVING_TO_POSE", "AWAITING_PULL", "RELEASING"))


class RequestsAreRejectedUnlessLegitimate(unittest.TestCase):
    def test_rejected_when_nothing_is_held(self):
        v = hs.HandoverState().request_release(1.0)
        self.assertFalse(v.ok)
        self.assertIn("no object is recorded as held", v.reason)

    def test_rejected_while_still_moving_to_the_pose(self):
        """The detector must never be armed during arm movement."""
        state = hs.HandoverState()
        state.begin("right", "bottle", True)
        v = state.request_release(1.0)
        self.assertFalse(v.ok)
        self.assertIn("not waiting for a handover", v.reason)
        self.assertIn("MOVING_TO_POSE", v.reason)

    def test_rejected_when_release_is_not_authorized(self):
        state = holding_and_awaiting(authorized=False)
        v = state.request_release(11.0)
        self.assertFalse(v.ok)
        self.assertIn("release authorization absent", v.reason)
        self.assertIn("detection-only", v.reason)

    def test_rejected_as_duplicate_while_one_is_pending(self):
        state = holding_and_awaiting()
        self.assertTrue(state.request_release(11.0).ok)
        v = state.request_release(11.1)
        self.assertFalse(v.ok)
        self.assertIn("duplicate", v.reason)
        self.assertIn("already pending", v.reason)

    def test_rejected_as_duplicate_after_one_was_consumed(self):
        state = holding_and_awaiting()
        state.request_release(11.0)
        self.assertIsNotNone(state.consume_pending(11.2))
        v = state.request_release(11.3)
        self.assertFalse(v.ok)
        self.assertIn("duplicate", v.reason)
        self.assertIn("already accepted", v.reason)

    def test_rejected_as_stale_after_the_awaiting_window(self):
        state = hs.HandoverState(max_awaiting_sec=5.0)
        state.begin("right", "bottle", True)
        state.mark_awaiting_pull(100.0)
        v = state.request_release(200.0)
        self.assertFalse(v.ok)
        self.assertIn("stale", v.reason)

    def test_rejected_after_release_already_happened(self):
        state = holding_and_awaiting()
        state.request_release(11.0)
        state.consume_pending(11.1)
        state.mark_released()
        v = state.request_release(12.0)
        self.assertFalse(v.ok)
        self.assertIn("no object is recorded as held", v.reason)

    def test_rejected_after_abandon(self):
        state = holding_and_awaiting()
        state.abandon("timeout")
        v = state.request_release(12.0)
        self.assertFalse(v.ok)
        self.assertIn("not waiting", v.reason)

    def test_a_rejected_request_records_nothing(self):
        state = holding_and_awaiting(authorized=False)
        state.request_release(11.0)
        self.assertFalse(state.snapshot().pending)
        self.assertIsNone(state.consume_pending(11.1))


class AcceptedRequestsAreRecordedNotExecuted(unittest.TestCase):
    def test_a_valid_request_is_accepted_and_recorded(self):
        state = holding_and_awaiting()
        v = state.request_release(11.0)
        self.assertTrue(v.ok, v.reason)
        self.assertEqual(v.sequence, 1)
        self.assertTrue(state.snapshot().pending)
        self.assertIn("the owning task will perform the release", v.reason)

    def test_the_task_consumes_the_event_exactly_once(self):
        state = holding_and_awaiting()
        state.request_release(11.0)
        event = state.consume_pending(11.2)
        self.assertIsNotNone(event)
        self.assertEqual(event.sequence, 1)
        self.assertEqual(event.side, "right")
        self.assertIsNone(state.consume_pending(11.3))

    def test_consuming_moves_the_task_into_releasing(self):
        state = holding_and_awaiting()
        state.request_release(11.0)
        state.consume_pending(11.2)
        self.assertEqual(state.snapshot().phase, hs.RELEASING)

    def test_a_new_session_allows_a_new_release(self):
        state = holding_and_awaiting()
        state.request_release(11.0)
        state.consume_pending(11.1)
        state.mark_released()
        state.begin("right", "cup", True)          # next handover
        state.mark_awaiting_pull(20.0)
        self.assertTrue(state.request_release(21.0).ok)

    def test_nothing_in_this_module_performs_io(self):
        with open(os.path.join(HERE, "..", "src", "orchestrator", "handover_state.py")) as f:
            code = f.read()
        for forbidden in ("rospy", "ServiceProxy", "/hand_", "release()", "subprocess", "socket"):
            self.assertNotIn(forbidden, code)


class NoDeadlockAndNoLockBypass(unittest.TestCase):
    """The whole point: validation must work WHILE the task holds the lock."""

    def test_a_request_succeeds_while_the_task_holds_the_execution_lock(self):
        lock = ExecutionLock()
        state = holding_and_awaiting()
        with lock.hold("deliver (awaiting pull)"):
            self.assertTrue(lock.busy)
            v = state.request_release(11.0)      # must not block, must not be refused
            self.assertTrue(v.ok, v.reason)
        self.assertFalse(lock.busy)

    def test_validation_returns_promptly_while_the_lock_is_held(self):
        lock = ExecutionLock()
        state = holding_and_awaiting()
        with lock.hold("deliver"):
            start = time.time()
            state.request_release(11.0)
            self.assertLess(time.time() - start, 1.0, "validation appears to be blocking")

    def test_reacquiring_the_execution_lock_would_have_been_refused(self):
        """Documents the bug this design avoids."""
        lock = ExecutionLock()
        with lock.hold("deliver (awaiting pull)"):
            with self.assertRaises(Busy):
                with lock.hold("auto_release"):
                    self.fail("a nested route must not be entered")

    def test_concurrent_requests_accept_exactly_one(self):
        state = holding_and_awaiting()
        results, errors = [], []

        def worker():
            try:
                results.append(state.request_release(11.0))
            except Exception as exc:              # noqa: BLE001
                errors.append(exc)

        threads = [threading.Thread(target=worker) for _ in range(12)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5)
        self.assertEqual(errors, [])
        self.assertEqual(len(results), 12)
        self.assertEqual(sum(1 for r in results if r.ok), 1, "exactly one must be accepted")

    def test_the_task_side_and_monitor_side_do_not_deadlock_each_other(self):
        state = holding_and_awaiting()
        stop = threading.Event()

        def consumer():
            while not stop.is_set():
                state.consume_pending(11.0)

        thread = threading.Thread(target=consumer)
        thread.start()
        try:
            for _ in range(200):
                state.request_release(11.0)
                state.snapshot()
        finally:
            stop.set()
            thread.join(timeout=5)
        self.assertFalse(thread.is_alive(), "consumer thread did not finish")


def executable_source(path):
    with open(path) as f:
        raw = f.read()
    kept = [t for t in tokenize.generate_tokens(io.StringIO(raw).readline)
            if t.type != tokenize.COMMENT]
    stripped = tokenize.untokenize(kept)
    tree = ast.parse(stripped)
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node, clean=False)
            if doc:
                stripped = stripped.replace(doc, "")
    return stripped


class ReleaseHandlerDoesNotTouchTheLock(unittest.TestCase):
    def setUp(self):
        self.code = executable_source(NODE)
        tree = ast.parse(self.code)
        self.handler = None
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == "handle_release_request":
                self.handler = node
        self.assertIsNotNone(self.handler, "handle_release_request is missing")

    def test_the_handler_never_references_the_execution_lock(self):
        """AST-precise: substring matching would trip over 'blockers'."""
        attrs, calls, names = set(), set(), set()
        for node in ast.walk(self.handler):
            if isinstance(node, ast.Attribute):
                attrs.add(node.attr)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                calls.add(node.func.attr)
            if isinstance(node, ast.Name):
                names.add(node.id)
        self.assertNotIn("lock", attrs, "handler reaches for self.lock")
        for forbidden in ("hold", "acquire", "release"):
            self.assertNotIn(forbidden, calls,
                             "handler calls execution-lock method %r" % forbidden)
        self.assertNotIn("Busy", names, "handler handles Busy, so it must be taking the lock")
        self.assertNotIn("ExecutionLock", names)

    def test_the_handler_never_runs_a_route(self):
        body = ast.dump(self.handler)
        for forbidden in ("run_route", "_execute", "resolve_route", "DryRunBackend", "RosBackend"):
            self.assertNotIn(forbidden, body)

    def test_the_handler_cannot_issue_motion(self):
        body = ast.dump(self.handler)
        self.assertNotIn("ServiceProxy", body)
        self.assertIn("motion_issued=False", self.code.replace(" ", "").replace(
            "motion_issued=False", "motion_issued=False"))

    def test_the_release_service_is_bound_to_the_handler_not_to_run(self):
        import re
        m = re.search(
            r'rospy\.Service\(\s*"/orchestrator/run_auto_release"\s*,\s*RunRoute\s*,\s*([^)]+)\)',
            self.code)
        self.assertIsNotNone(m, "run_auto_release service registration not found")
        self.assertIn("handle_release_request", m.group(1))
        self.assertNotIn("make_handler", m.group(1))

    def test_the_other_three_routes_still_go_through_the_lock(self):
        self.assertIn("make_handler", self.code)
        for route in ("run_navigate_report", "run_grasp", "run_deliver"):
            self.assertIn(route, self.code)


if __name__ == "__main__":
    unittest.main()
