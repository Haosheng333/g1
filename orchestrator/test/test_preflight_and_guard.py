"""Preflight blocker collection, the saw_active guard, and the execution lock."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from orchestrator import capabilities as caps  # noqa: E402
from orchestrator import nav_guard as ng  # noqa: E402
from orchestrator import preflight as pf  # noqa: E402
from orchestrator import routes as r  # noqa: E402
from orchestrator.execution_lock import Busy, ExecutionLock  # noqa: E402


class FakeProbe(pf.Probe):
    """Everything present and true unless explicitly withheld."""

    def __init__(self, missing_services=(), flags=None, missing_topics=(), stale=()):
        self.missing_services = set(missing_services)
        self.flags = {} if flags is None else dict(flags)
        self.missing_topics = set(missing_topics)
        self.stale = set(stale)
        self.service_queries = []

    def has_service(self, name):
        self.service_queries.append(name)
        return name not in self.missing_services

    def flag(self, topic):
        return self.flags.get(topic, True)

    def has_publisher(self, topic):
        return topic not in self.missing_topics

    def fresh(self, topic, max_age_s):
        if topic in self.stale:
            return "%s is stale" % topic
        if topic in self.missing_topics:
            return "%s not received (WAIT)" % topic
        return ""


class PreflightHappyPath(unittest.TestCase):
    def test_everything_available_passes(self):
        result = pf.run_preflight(r.NAVIGATE_REPORT, FakeProbe(), "right")
        self.assertTrue(result.ok, result.summary())
        self.assertEqual(result.blockers, [])

    def test_hand_services_are_resolved_for_the_configured_side(self):
        probe = FakeProbe()
        pf.run_preflight(r.AUTO_RELEASE, probe, "left")
        self.assertIn("/hand_left/release", probe.service_queries)
        self.assertNotIn("/hand_right/release", probe.service_queries)

    def test_a_check_is_produced_for_every_requirement(self):
        result = pf.run_preflight(r.NAVIGATE_REPORT, FakeProbe(), "right")
        self.assertEqual(
            [c.capability for c in result.checks], list(r.NAVIGATE_REPORT.requires)
        )


class PreflightCollectsEveryBlocker(unittest.TestCase):
    def test_all_blockers_are_reported_not_just_the_first(self):
        probe = FakeProbe(
            missing_services=["/navigation/navigate", "/perception/get_object_position"],
            flags={"/slam/manager/state": False},
            missing_topics=["/map"],
            stale=["/joint_states"],
        )
        result = pf.run_preflight(r.NAVIGATE_REPORT, probe, "right")
        self.assertFalse(result.ok)
        joined = " | ".join(result.blockers)
        for expected in ("/navigation/navigate", "/perception/get_object_position",
                         "/slam/manager/state", "/map", "/joint_states"):
            self.assertIn(expected, joined)
        self.assertGreaterEqual(len(result.blockers), 5)

    def test_a_never_received_flag_is_a_wait_not_a_false(self):
        probe = FakeProbe(flags={})
        probe.flag = lambda topic: None
        result = pf.run_preflight(r.NAVIGATE_REPORT, probe, "right")
        self.assertFalse(result.ok)
        self.assertIn("WAIT", " ".join(result.blockers))

    def test_a_false_flag_is_distinguished_from_a_missing_one(self):
        probe = FakeProbe(flags={"/slam/manager/state": False})
        result = pf.run_preflight(r.NAVIGATE_REPORT, probe, "right")
        self.assertIn("is false", " ".join(result.blockers))

    def test_each_blocker_names_the_owning_package(self):
        probe = FakeProbe(missing_services=["/navigation/navigate"])
        result = pf.run_preflight(r.NAVIGATE_REPORT, probe, "right")
        self.assertIn("(navigation)", " ".join(result.blockers))

    def test_blockers_mention_every_missing_arm_planning_service(self):
        probe = FakeProbe(missing_services=["/arm/plan_to_pose", "/arm/plan_grasp_from_point"])
        check = pf.check_capability(caps.ARM_PLANNING, probe, "right")
        self.assertFalse(check.ok)
        self.assertIn("/arm/plan_to_pose", check.detail)
        self.assertIn("/arm/plan_grasp_from_point", check.detail)

    def test_summary_is_readable_in_both_outcomes(self):
        self.assertIn("preflight OK", pf.run_preflight(r.AUTO_RELEASE, FakeProbe(), "right").summary())
        bad = pf.run_preflight(r.AUTO_RELEASE, FakeProbe(missing_services=["/hand_right/grasp"]), "right")
        self.assertIn("preflight FAILED", bad.summary())


class UnresolvedCapability(unittest.TestCase):
    """An unresolved decision is a blocker no amount of running hardware clears."""

    def test_frame_contract_fails_even_with_a_perfect_probe(self):
        check = pf.check_capability(caps.FRAME_CONTRACT, FakeProbe(), "right")
        self.assertFalse(check.ok)
        self.assertTrue(check.unresolved)

    def test_grasp_preflight_fails_with_everything_else_available(self):
        result = pf.run_preflight(r.GRASP, FakeProbe(), "right")
        self.assertFalse(result.ok)
        self.assertEqual(len(result.blockers), 1, result.blockers)
        self.assertEqual(result.unresolved_blockers, result.blockers)

    def test_unresolved_blockers_are_separable_from_runtime_ones(self):
        probe = FakeProbe(missing_services=["/hand_right/grasp"])
        result = pf.run_preflight(r.GRASP, probe, "right")
        self.assertEqual(len(result.unresolved_blockers), 1)
        self.assertGreater(len(result.blockers), len(result.unresolved_blockers))


class SawActiveGuard(unittest.TestCase):
    """/navigation/status is latched, so a stale terminal state must not count."""

    def test_stale_success_before_any_active_is_ignored(self):
        guard = ng.NavGuard(timeout_s=10.0)
        guard.start(0.0)
        self.assertEqual(guard.update(0.1, "SUCCESS"), ng.PENDING)
        self.assertFalse(guard.saw_active)
        self.assertIn("stale", guard.reason)

    def test_success_after_active_is_accepted(self):
        guard = ng.NavGuard(timeout_s=10.0)
        guard.start(0.0)
        guard.update(0.1, "SUCCESS")        # stale, ignored
        guard.update(0.2, "FOLLOWING")
        self.assertEqual(guard.update(0.3, "SUCCESS"), ng.SUCCESS)

    def test_failure_after_active_is_a_failure(self):
        guard = ng.NavGuard(timeout_s=10.0)
        guard.start(0.0)
        guard.update(0.1, "PLANNING")
        self.assertEqual(guard.update(0.2, "FAILURE"), ng.FAILURE)

    def test_every_active_state_arms_the_guard(self):
        for active in ng.ACTIVE:
            guard = ng.NavGuard(timeout_s=10.0)
            guard.start(0.0)
            guard.update(0.1, active)
            self.assertTrue(guard.saw_active, active)
            self.assertEqual(guard.update(0.2, "SUCCESS"), ng.SUCCESS, active)

    def test_timeout_without_any_status(self):
        guard = ng.NavGuard(timeout_s=1.0)
        guard.start(0.0)
        self.assertEqual(guard.update(0.5, None), ng.PENDING)
        self.assertEqual(guard.update(1.5, None), ng.TIMEOUT)
        self.assertIn("saw_active=False", guard.reason)

    def test_timeout_while_still_active(self):
        guard = ng.NavGuard(timeout_s=1.0)
        guard.start(0.0)
        guard.update(0.5, "FOLLOWING")
        self.assertEqual(guard.update(1.5, "FOLLOWING"), ng.TIMEOUT)
        self.assertIn("saw_active=True", guard.reason)

    def test_a_stale_success_cannot_be_rescued_by_a_timeout(self):
        """Never report SUCCESS for a goal that was never observed active."""
        guard = ng.NavGuard(timeout_s=1.0)
        guard.start(0.0)
        for t in (0.2, 0.4, 0.6, 0.8, 1.2):
            verdict = guard.update(t, "SUCCESS")
        self.assertEqual(verdict, ng.TIMEOUT)

    def test_verdict_is_sticky(self):
        guard = ng.NavGuard(timeout_s=10.0)
        guard.start(0.0)
        guard.update(0.1, "FOLLOWING")
        self.assertEqual(guard.update(0.2, "SUCCESS"), ng.SUCCESS)
        self.assertEqual(guard.update(0.3, "FAILURE"), ng.SUCCESS)

    def test_unknown_status_is_neither_active_nor_terminal(self):
        guard = ng.NavGuard(timeout_s=10.0)
        guard.start(0.0)
        self.assertEqual(guard.update(0.1, "WAT"), ng.PENDING)
        self.assertFalse(guard.saw_active)

    def test_update_before_start_raises(self):
        with self.assertRaises(RuntimeError):
            ng.NavGuard(timeout_s=1.0).update(0.0, "FOLLOWING")

    def test_non_positive_timeout_is_rejected(self):
        for bad in (0.0, -1.0):
            with self.assertRaises(ValueError):
                ng.NavGuard(timeout_s=bad)

    def test_active_and_terminal_sets_match_navigation(self):
        self.assertEqual(ng.ACTIVE, ("PLANNING", "FOLLOWING", "RECOVERING"))
        self.assertEqual(ng.TERMINAL, ("SUCCESS", "FAILURE"))


class NonBlockingLock(unittest.TestCase):
    def test_second_acquire_is_refused_immediately(self):
        lock = ExecutionLock()
        self.assertTrue(lock.acquire("grasp"))
        self.assertFalse(lock.acquire("deliver"))
        self.assertEqual(lock.holder, "grasp")

    def test_release_allows_the_next_route(self):
        lock = ExecutionLock()
        lock.acquire("grasp")
        lock.release()
        self.assertFalse(lock.busy)
        self.assertTrue(lock.acquire("deliver"))

    def test_hold_raises_busy_rather_than_waiting(self):
        lock = ExecutionLock()
        with lock.hold("grasp"):
            with self.assertRaises(Busy) as ctx:
                with lock.hold("deliver"):
                    self.fail("should not have been entered")
            self.assertIn("grasp", str(ctx.exception))

    def test_hold_releases_on_exception(self):
        lock = ExecutionLock()
        try:
            with lock.hold("grasp"):
                raise RuntimeError("boom")
        except RuntimeError:
            pass
        self.assertFalse(lock.busy)
        self.assertTrue(lock.acquire("deliver"))

    def test_release_without_holding_is_harmless(self):
        lock = ExecutionLock()
        lock.release()
        self.assertFalse(lock.busy)


if __name__ == "__main__":
    unittest.main()
