"""Sequencer: dry-run traversal, fail-fast, retries, and unconditional cleanup."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from orchestrator import routes as r  # noqa: E402
from orchestrator import sequencer as seq  # noqa: E402


def ctx(route="navigate_report", target="table", dry_run=True):
    return seq.RouteContext(route=route, target=target, side="right", dry_run=dry_run)


class RecordingBackend(seq.DryRunBackend):
    """A dry-run backend with scripted failures, for fail-fast and retry tests."""

    def __init__(self, fail=None, fail_times=None):
        super(RecordingBackend, self).__init__()
        self.fail = set(fail or ())
        self.fail_times = dict(fail_times or {})
        self.attempts = {}

    def _record(self, action, detail=""):
        self.attempts[action] = self.attempts.get(action, 0) + 1
        if action in self.fail_times:
            if self.attempts[action] <= self.fail_times[action]:
                self.calls.append(action)
                return seq.StepResult(False, "scripted transient failure of %s" % action)
        if action in self.fail:
            self.calls.append(action)
            return seq.StepResult(False, "scripted failure of %s" % action)
        return super(RecordingBackend, self)._record(action, detail)


class DryRunTraversal(unittest.TestCase):
    def test_dry_run_traverses_every_step(self):
        backend = seq.DryRunBackend()
        result = seq.run_route(r.NAVIGATE_REPORT, backend, ctx())
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.steps_completed, r.NAVIGATE_REPORT.step_names)

    def test_dry_run_issues_no_motion(self):
        for route in r.ROUTES.values():
            backend = seq.DryRunBackend()
            result = seq.run_route(route, backend, ctx(route=route.name))
            self.assertFalse(result.motion_issued, route.name)

    def test_dry_run_traverses_the_blocked_grasp_route_too(self):
        """Blockers are the caller's decision; traversal itself must still work."""
        backend = seq.DryRunBackend()
        result = seq.run_route(r.GRASP, backend, ctx(route="grasp"))
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.steps_completed, r.GRASP.step_names)
        self.assertIn("plan_grasp", backend.calls)
        self.assertIn("execute_to_pregrasp", backend.calls)

    def test_dry_run_backend_builds_no_client(self):
        """DryRunBackend must have no attribute that looks like a proxy."""
        backend = seq.DryRunBackend()
        seq.run_route(r.GRASP, backend, ctx(route="grasp"))
        for name, value in vars(backend).items():
            self.assertIn(name, ("calls", "cleanup", "_t"), "unexpected state %r" % name)

    def test_every_action_in_every_route_is_implemented(self):
        backend = seq.DryRunBackend()
        for route in r.ROUTES.values():
            for step in route.steps:
                self.assertTrue(
                    hasattr(backend, step.action),
                    "%s/%s needs backend.%s" % (route.name, step.name, step.action),
                )

    def test_base_backend_declares_every_action_used(self):
        for route in r.ROUTES.values():
            for step in route.steps:
                self.assertTrue(hasattr(seq.Backend, step.action), step.action)


class BlockersStopEverything(unittest.TestCase):
    def test_one_blocker_means_zero_steps(self):
        backend = seq.DryRunBackend()
        result = seq.run_route(
            r.NAVIGATE_REPORT, backend, ctx(), blockers=["localization (slam): not localized"]
        )
        self.assertFalse(result.success)
        self.assertEqual(result.steps_completed, [])
        self.assertEqual(backend.calls, [])
        self.assertIn("NOT_READY", result.message)

    def test_blockers_are_passed_through_verbatim(self):
        blockers = ["a (x): one", "b (y): two"]
        result = seq.run_route(r.NAVIGATE_REPORT, seq.DryRunBackend(), ctx(), blockers=blockers)
        self.assertEqual(result.blockers, blockers)

    def test_cleanup_still_runs_when_blocked(self):
        """Even refusing to start must leave the gate shut."""
        backend = seq.DryRunBackend()
        seq.run_route(r.NAVIGATE_REPORT, backend, ctx(), blockers=["x"])
        self.assertEqual(backend.cleanup, [])

    def test_blocked_route_reports_skipped_steps_from_resume(self):
        result = seq.run_route(
            r.NAVIGATE_REPORT, seq.DryRunBackend(), ctx(),
            start_step="detect_object", blockers=["x"],
        )
        self.assertEqual(
            result.steps_skipped, ["await_localization", "enable_motion", "navigate"]
        )


class FailFast(unittest.TestCase):
    def test_a_failing_step_stops_the_route(self):
        backend = RecordingBackend(fail={"navigate"})
        result = seq.run_route(r.NAVIGATE_REPORT, backend, ctx())
        self.assertFalse(result.success)
        self.assertEqual(result.final_step, "navigate")
        self.assertEqual(result.steps_completed, ["await_localization", "enable_motion"])
        self.assertNotIn("detect_object", backend.calls)

    def test_a_raising_step_is_a_failed_step_not_a_crash(self):
        class Boom(seq.DryRunBackend):
            def navigate(self, ctx):
                raise RuntimeError("nav exploded")

        backend = Boom()
        result = seq.run_route(r.NAVIGATE_REPORT, backend, ctx())
        self.assertFalse(result.success)
        self.assertIn("RuntimeError", result.message)
        self.assertIn("nav exploded", result.message)
        self.assertEqual(backend.cleanup,
                         ["cancel_navigation", "disable_motion", "disarm_monitor_cleanup"])

    def test_a_missing_backend_action_is_a_failed_step(self):
        """A backend that does not implement a route's action must not crash."""
        class Partial(seq.DryRunBackend):
            report = None       # shadows the inherited implementation

        route = r.Route(name="t", steps=(r.Step("report", "report"),))
        result = seq.run_route(route, Partial(), ctx())
        self.assertFalse(result.success)
        self.assertIn("no action", result.message)

    def test_an_action_absent_from_the_backend_entirely_is_a_failed_step(self):
        route = r.Route(name="t", steps=(r.Step("x", "not_a_real_action"),))
        result = seq.run_route(route, seq.DryRunBackend(), ctx())
        self.assertFalse(result.success)
        self.assertIn("not_a_real_action", result.message)

    def test_unknown_start_step_fails_before_any_step(self):
        backend = seq.DryRunBackend()
        result = seq.run_route(r.NAVIGATE_REPORT, backend, ctx(), start_step="nope")
        self.assertFalse(result.success)
        self.assertEqual(backend.calls, [])
        self.assertIn("nope", result.message)


class Retries(unittest.TestCase):
    def test_unlisted_actions_get_exactly_one_attempt(self):
        backend = RecordingBackend(fail={"navigate"})
        seq.run_route(r.NAVIGATE_REPORT, backend, ctx(), retry=seq.RetryPolicy())
        self.assertEqual(backend.attempts["navigate"], 1)

    def test_a_configured_action_is_retried_and_can_recover(self):
        backend = RecordingBackend(fail_times={"detect_object": 2})
        policy = seq.RetryPolicy(attempts={"detect_object": 3})
        result = seq.run_route(r.NAVIGATE_REPORT, backend, ctx(), retry=policy)
        self.assertTrue(result.success, result.message)
        self.assertEqual(backend.attempts["detect_object"], 3)

    def test_retries_are_exhausted_then_the_route_fails(self):
        backend = RecordingBackend(fail={"detect_object"})
        policy = seq.RetryPolicy(attempts={"detect_object": 2})
        result = seq.run_route(r.NAVIGATE_REPORT, backend, ctx(), retry=policy)
        self.assertFalse(result.success)
        self.assertEqual(backend.attempts["detect_object"], 2)
        self.assertEqual(result.final_step, "detect_object")

    def test_attempts_below_one_are_clamped(self):
        self.assertEqual(seq.RetryPolicy(attempts={"x": 0}).for_action("x"), 1)
        self.assertEqual(seq.RetryPolicy(attempts={"x": -5}).for_action("x"), 1)

    def test_failed_attempts_are_noted_in_the_context(self):
        backend = RecordingBackend(fail_times={"detect_object": 1})
        context = ctx()
        seq.run_route(
            r.NAVIGATE_REPORT, backend, context, retry=seq.RetryPolicy(attempts={"detect_object": 2})
        )
        self.assertTrue(any("detect_object attempt 1/2" in n for n in context.notes))


class UnconditionalCleanup(unittest.TestCase):
    EXPECTED = ["cancel_navigation", "disable_motion", "disarm_monitor_cleanup"]

    def test_cleanup_after_success(self):
        backend = seq.DryRunBackend()
        seq.run_route(r.NAVIGATE_REPORT, backend, ctx())
        self.assertEqual(backend.cleanup, self.EXPECTED)

    def test_cleanup_after_failure(self):
        backend = RecordingBackend(fail={"navigate"})
        seq.run_route(r.NAVIGATE_REPORT, backend, ctx())
        self.assertEqual(backend.cleanup, self.EXPECTED)

    def test_cleanup_after_a_raising_step(self):
        class Boom(seq.DryRunBackend):
            def report(self, ctx):
                raise ValueError("x")

        backend = Boom()
        seq.run_route(r.NAVIGATE_REPORT, backend, ctx())
        self.assertEqual(backend.cleanup, self.EXPECTED)

    def test_cleanup_runs_for_every_route(self):
        for route in r.ROUTES.values():
            backend = seq.DryRunBackend()
            seq.run_route(route, backend, ctx(route=route.name))
            self.assertEqual(backend.cleanup, self.EXPECTED, route.name)

    def test_cleanup_runs_even_when_it_is_the_only_thing_left(self):
        backend = RecordingBackend(fail={"await_localization"})
        seq.run_route(r.NAVIGATE_REPORT, backend, ctx())
        self.assertEqual(backend.cleanup, self.EXPECTED)


class ResumeExecution(unittest.TestCase):
    def test_resume_runs_only_the_remaining_steps(self):
        backend = seq.DryRunBackend()
        result = seq.run_route(r.NAVIGATE_REPORT, backend, ctx(), start_step="detect_object")
        self.assertTrue(result.success)
        self.assertEqual(result.steps_completed, ["detect_object", "report"])
        self.assertEqual(
            result.steps_skipped, ["await_localization", "enable_motion", "navigate"]
        )
        self.assertNotIn("navigate", backend.calls)

    def test_resume_past_the_motion_gate_does_not_reopen_it(self):
        backend = seq.DryRunBackend()
        seq.run_route(r.NAVIGATE_REPORT, backend, ctx(), start_step="detect_object")
        self.assertNotIn("enable_motion", backend.calls)


if __name__ == "__main__":
    unittest.main()
