"""Integration tests of the PRODUCTION delivery route.

These drive the real `sequencer.run_route()` over the real `routes.DELIVER`,
with a fake backend, an injected clock and injected pull events. The fake
backend deliberately does NOT override `await_pull`, so the shared production
polling loop in `sequencer.Backend.await_pull` is the code under test.

ALL DATA HERE IS SYNTHETIC AND TEST-ONLY. No ROS, no hardware, no clock, no
sleeping. Nothing here is hardware validation.
"""
import os
import sys
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from orchestrator import handover_state as hs  # noqa: E402
from orchestrator import routes as r  # noqa: E402
from orchestrator import sequencer as seq  # noqa: E402
from orchestrator.execution_lock import ExecutionLock  # noqa: E402

#: The delivery steps, in the order the framework specifies.
EXPECTED_ORDER = [
    "require_held_object", "await_localization", "enable_motion", "navigate",
    "plan_handover_pose", "execute_handover_pose", "settle_handover",
    "arm_monitor", "await_pull", "disarm_monitor", "release_hand",
    "confirm_released", "settle_recipient", "execute_retract_home",
]


class FakeBackend(seq.Backend):
    """Records calls, performs nothing, and runs the REAL await_pull loop.

    `pull_after_sleeps` injects a validated pull into the task record after N
    poll iterations, standing in for a human taking the object. `fail` and
    `release_result` script failures. `cancel_after_sleeps` simulates a cancel.
    """

    def __init__(self, fail=(), release_result="ok", pull_after_sleeps=None,
                 cancel_after_sleeps=None, token_override=None, clock=1000.0,
                 exposes_settling=False):
        self.calls = []
        self.cleanup = []
        self.motion = []
        self._t = float(clock)
        self._sleeps = 0
        self.fail = set(fail)
        self.release_result = release_result
        self.pull_after_sleeps = pull_after_sleeps
        self.cancel_after_sleeps = cancel_after_sleeps
        self.token_override = token_override
        #: Models a FUTURE hand_control that exposes settling in its response
        #: (a structured flag, or the feedback positions). Not available today;
        #: see the release_completion capability.
        self.exposes_settling = bool(exposes_settling)
        self._cancelled = False
        self._ctx = None
        self.await_pull_timeout_sec = 5.0
        self.await_pull_poll_sec = 0.1

    # --- injected clock --------------------------------------------------------
    def now(self):
        return self._t

    def sleep(self, seconds):
        self._t += float(seconds)
        self._sleeps += 1
        if self.cancel_after_sleeps is not None and self._sleeps >= self.cancel_after_sleeps:
            self._cancelled = True
        if self.pull_after_sleeps is not None and self._sleeps == self.pull_after_sleeps:
            self._inject_pull()

    def aborted(self):
        return self._cancelled

    def await_pull(self, ctx):
        """Record the call, then run the PRODUCTION loop unchanged."""
        self.calls.append("await_pull")
        return seq.Backend.await_pull(self, ctx)

    def _inject_pull(self, token=None):
        """Stand in for the monitor's accepted request arriving mid-wait."""
        tok = token if token is not None else (
            self.token_override if self.token_override is not None else self._ctx.session_token)
        return self._ctx.handover.request_release(self.now(), "test", event_token=tok)

    # --- step helpers ----------------------------------------------------------
    def _record(self, name, detail="", motion=False):
        self.calls.append(name)
        if motion:
            self.motion.append(name)
        if name in self.fail:
            return seq.StepResult(False, "scripted failure of %s" % name)
        return seq.StepResult(True, detail or name, motion_issued=motion)

    # --- actions ---------------------------------------------------------------
    def require_held_object(self, ctx):
        self._ctx = ctx
        self.calls.append("require_held_object")
        if "require_held_object" in self.fail:
            return seq.StepResult(False, "scripted failure")
        v = ctx.handover.require_held_object(ctx.side)
        return seq.StepResult(v.ok, v.reason)

    def record_held_object(self, ctx):
        self._ctx = ctx
        self.calls.append("record_held_object")
        ctx.handover.begin(ctx.side, ctx.target or "object", release_authorized=True)
        ctx.held_label = ctx.target or "object"
        return seq.StepResult(True, "recorded")

    def await_localization(self, ctx):
        self._ctx = ctx
        return self._record("await_localization")

    def enable_motion(self, ctx):
        return self._record("enable_motion", motion=True)

    def navigate(self, ctx):
        return self._record("navigate", motion=True)

    def detect_object(self, ctx):
        return self._record("detect_object")

    def report(self, ctx):
        return self._record("report")

    def to_planner_frame(self, ctx):
        return self._record("to_planner_frame")

    def plan_grasp(self, ctx):
        ctx.grasp_final_joints = [0.1, 0.2, 0.3, -0.4, 0.5]
        return self._record("plan_grasp")

    def plan_stow(self, ctx):
        """Spec 2.4: planned BEFORE the grasp moves, from the retract end pose."""
        return self._record("plan_stow")

    def plan_handover_pose(self, ctx):
        return self._record("plan_handover_pose")

    def execute_segment(self, ctx, step):
        return self._record(step.name, motion=True)

    def open_gripper(self, ctx):
        return self._record("open_gripper", motion=True)

    def close_gripper(self, ctx):
        return self._record("close_gripper", motion=True)

    def settle_handover(self, ctx):
        return self._record("settle_handover")

    def settle_recipient(self, ctx):
        return self._record("settle_recipient")

    def arm_monitor(self, ctx):
        self._ctx = ctx
        self.calls.append("arm_monitor")
        if "arm_monitor" in self.fail:
            return seq.StepResult(False, "scripted failure")
        token = hs.new_session_token()
        ctx.handover.begin_arming(self.now(), token)     # window opens
        ctx.session_token = token
        if "arm_monitor_echo" in self.fail:
            ctx.handover.abandon("token echo mismatch")
            return seq.StepResult(False, "monitor echoed the wrong token; abandoned")
        ctx.handover.mark_awaiting_pull(self.now(), token)   # window closes
        return seq.StepResult(True, "armed, token %s" % token)

    def disarm_monitor(self, ctx):
        return self._record("disarm_monitor")

    def release_hand(self, ctx):
        permitted, reason = ctx.handover.release_permitted()
        if not permitted:
            self.calls.append("release_hand_refused")
            return seq.StepResult(False, reason)
        self.calls.append("release_hand")
        self.motion.append("release_hand")
        ctx.release_outcome = self.release_result
        if self.release_result != "ok":
            ctx.handover.mark_release_uncertain(self.release_result)
            return seq.StepResult(False, "release %s" % self.release_result, motion_issued=True)
        return seq.StepResult(True, "release ok", motion_issued=True)

    def confirm_released(self, ctx):
        self.calls.append("confirm_released")
        if ctx.release_outcome == "ok":
            if self.exposes_settling:
                ctx.handover.mark_release_settled(
                    "test fake: structured settled flag in the release response")
                return seq.StepResult(True, "fingers settled; TRANSFER STILL NOT VERIFIED")
            ctx.handover.mark_release_commanded()
            return seq.StepResult(True, "command acknowledged only")
        ctx.handover.mark_release_uncertain(ctx.release_outcome)
        return seq.StepResult(False, "UNCERTAIN: %s" % ctx.release_outcome)

    def execute_retract_home(self, ctx):
        self.calls.append("execute_retract_home")
        allowed, reason = ctx.handover.retraction_allowed()
        if not allowed:
            return seq.StepResult(False, reason)
        self.motion.append("execute_retract_home")
        # End of delivery: the task record is closed here.
        ctx.handover.mark_done()
        return seq.StepResult(True, reason, motion_issued=True)

    def execute_stow(self, ctx):
        return self._record("execute_stow", motion=True)

    # --- cleanup ---------------------------------------------------------------
    def cancel_navigation(self):
        self.cleanup.append("cancel_navigation")

    def disable_motion(self):
        self.cleanup.append("disable_motion")

    def disarm_monitor_cleanup(self):
        self.cleanup.append("disarm_monitor_cleanup")


def held_state(side="right", label="bottle", authorized=True):
    """A task record already holding an object, as a grasp would leave it."""
    state = hs.HandoverState()
    state.begin(side, label, release_authorized=authorized)
    return state


def deliver(backend, state, start_step="", side="right", target="person"):
    ctx = seq.RouteContext(route="deliver", target=target, side=side,
                           dry_run=False, handover=state)
    return seq.run_route(r.DELIVER, backend, ctx, start_step, blockers=[]), ctx


class SuccessfulSequence(unittest.TestCase):
    def test_every_step_is_attempted_in_the_specified_order(self):
        """All 14 steps run; retraction is the one that stops, by design."""
        backend = FakeBackend(pull_after_sleeps=2)
        result, ctx = deliver(backend, held_state())
        self.assertEqual(backend.calls, EXPECTED_ORDER)
        # Default: a bare command acknowledgement does not authorise retraction.
        self.assertFalse(result.success)
        self.assertEqual(result.final_step, "execute_retract_home")
        self.assertIn("NOT_READY", result.message)
        self.assertEqual(result.steps_completed, EXPECTED_ORDER[:-1])

    def test_the_sequence_completes_only_if_the_hand_exposes_settling(self):
        """Requires a hand_control that reports settling. It does not today."""
        backend = FakeBackend(pull_after_sleeps=2, exposes_settling=True)
        result, ctx = deliver(backend, held_state())
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.steps_completed, EXPECTED_ORDER)
        self.assertEqual(backend.calls, EXPECTED_ORDER)

    def test_retraction_is_refused_on_a_command_acknowledgement(self):
        backend = FakeBackend(pull_after_sleeps=2)
        result, ctx = deliver(backend, held_state())
        self.assertIn("execute_retract_home", backend.calls)
        self.assertNotIn("execute_retract_home", backend.motion,
                         "the arm must not move on a command ack alone")
        self.assertIn("require_settle", result.message)

    def test_disarm_happens_strictly_before_release(self):
        backend = FakeBackend(pull_after_sleeps=2)
        deliver(backend, held_state())
        self.assertLess(backend.calls.index("disarm_monitor"),
                        backend.calls.index("release_hand"))

    def test_the_detector_is_armed_only_after_the_arm_has_moved_and_settled(self):
        backend = FakeBackend(pull_after_sleeps=2)
        deliver(backend, held_state())
        i = backend.calls.index
        self.assertLess(i("execute_handover_pose"), i("settle_handover"))
        self.assertLess(i("settle_handover"), i("arm_monitor"))

    def test_the_pull_event_is_actually_consumed(self):
        backend = FakeBackend(pull_after_sleeps=2)
        result, ctx = deliver(backend, held_state())
        snap = ctx.handover.snapshot()
        self.assertTrue(snap.consumed_this_session, "consume_pending was never reached")
        self.assertFalse(snap.pending)

    def test_release_is_recorded_as_commanded_not_verified(self):
        """A successful release service call is an ack, not proof of transfer."""
        state = held_state()
        seen = {}
        # NOTE: no success assertion -- by default this route stops at retraction.

        class Peeking(FakeBackend):
            def settle_recipient(self, ctx):
                seen["at_settle"] = ctx.handover.object_state
                return FakeBackend.settle_recipient(self, ctx)

        backend = Peeking(pull_after_sleeps=2)
        result, ctx = deliver(backend, state)
        self.assertEqual(seen["at_settle"], hs.RELEASE_COMMANDED,
                         "a successful release must not be recorded as verified transfer")
        self.assertNotEqual(seen["at_settle"], hs.RELEASE_SETTLED)

    def test_a_completed_delivery_closes_the_task_record(self):
        backend = FakeBackend(pull_after_sleeps=2, exposes_settling=True)
        result, ctx = deliver(backend, held_state())
        self.assertTrue(result.success, result.message)
        self.assertEqual(ctx.handover.object_state, hs.NONE)
        self.assertFalse(ctx.handover.holding)
        self.assertEqual(ctx.handover.phase, hs.IDLE)

    def test_cleanup_always_runs(self):
        backend = FakeBackend(pull_after_sleeps=2)
        deliver(backend, held_state())
        self.assertEqual(backend.cleanup,
                         ["cancel_navigation", "disable_motion", "disarm_monitor_cleanup"])


class GraspStateCarriesIntoDelivery(unittest.TestCase):
    def test_grasp_records_the_held_object_and_delivery_reads_it(self):
        state = hs.HandoverState()
        grasp_backend = FakeBackend()
        ctx = seq.RouteContext(route="grasp", target="bottle", side="right",
                               dry_run=False, handover=state)
        grasp = seq.run_route(r.GRASP, grasp_backend, ctx, "", blockers=[])
        self.assertTrue(grasp.success, grasp.message)
        self.assertIn("record_held_object", grasp_backend.calls)
        self.assertEqual(state.object_state, hs.HELD)
        self.assertTrue(state.holding)

        result, _ = deliver(FakeBackend(pull_after_sleeps=2, exposes_settling=True), state)
        self.assertTrue(result.success, result.message)

    def test_delivery_refuses_without_a_grasp(self):
        result, _ = deliver(FakeBackend(pull_after_sleeps=2), hs.HandoverState())
        self.assertFalse(result.success)
        self.assertEqual(result.final_step, "require_held_object")
        self.assertIn("no object is recorded as held", result.message)
        self.assertEqual(result.steps_completed, [])

    def test_delivery_refuses_for_the_wrong_hand(self):
        result, _ = deliver(FakeBackend(pull_after_sleeps=2),
                            held_state(side="left"), side="right")
        self.assertFalse(result.success)
        self.assertIn("left hand, not right", result.message)


class EventConsumedUnderTheExecutionLock(unittest.TestCase):
    def test_the_event_is_consumed_while_the_route_owns_the_lock(self):
        lock = ExecutionLock()
        state = held_state()
        observed = {}

        class Observing(FakeBackend):
            def sleep(self, seconds):
                observed["busy"] = lock.busy
                observed["holder"] = lock.holder
                FakeBackend.sleep(self, seconds)

        backend = Observing(pull_after_sleeps=1, exposes_settling=True)
        with lock.hold("deliver (test)"):
            result, _ = deliver(backend, state)
        self.assertTrue(result.success, result.message)
        self.assertTrue(observed["busy"], "the lock must be held while awaiting the pull")
        self.assertEqual(observed["holder"], "deliver (test)")
        self.assertFalse(lock.busy)

    def test_validation_succeeds_while_the_lock_is_held(self):
        lock = ExecutionLock()
        state = held_state()
        token = hs.new_session_token()
        state.mark_awaiting_pull(1000.0, token)
        with lock.hold("deliver"):
            v = state.request_release(1000.1, "test", event_token=token)
        self.assertTrue(v.ok, v.reason)


class TimeoutAndCancelProduceNoRelease(unittest.TestCase):
    def test_timeout_produces_no_release_and_keeps_the_object(self):
        backend = FakeBackend(pull_after_sleeps=None)      # no pull ever arrives
        result, ctx = deliver(backend, held_state())
        self.assertFalse(result.success)
        self.assertEqual(result.final_step, "await_pull")
        self.assertIn("TIMEOUT", result.message)
        self.assertNotIn("release_hand", backend.calls)
        self.assertNotIn("execute_retract_home", backend.calls)
        self.assertEqual(ctx.handover.object_state, hs.HELD)

    def test_cancel_produces_no_release_and_keeps_the_object(self):
        backend = FakeBackend(cancel_after_sleeps=2)
        result, ctx = deliver(backend, held_state())
        self.assertFalse(result.success)
        self.assertIn("CANCELLED", result.message)
        self.assertNotIn("release_hand", backend.calls)
        self.assertEqual(ctx.handover.object_state, hs.HELD)

    def test_the_monitor_is_disarmed_even_on_timeout(self):
        backend = FakeBackend(pull_after_sleeps=None)
        deliver(backend, held_state())
        self.assertIn("disarm_monitor_cleanup", backend.cleanup)

    def test_timeout_issues_no_arm_or_hand_motion(self):
        backend = FakeBackend(pull_after_sleeps=None)
        deliver(backend, held_state())
        for forbidden in ("release_hand", "execute_retract_home"):
            self.assertNotIn(forbidden, backend.motion)


class ReleaseFailurePreservesStateAndBlocksRetraction(unittest.TestCase):
    def test_release_failure_marks_uncertain_and_refuses_retraction(self):
        backend = FakeBackend(pull_after_sleeps=2, release_result="serial timeout")
        result, ctx = deliver(backend, held_state())
        self.assertFalse(result.success)
        self.assertEqual(result.final_step, "release_hand")
        self.assertEqual(ctx.handover.object_state, hs.UNCERTAIN)
        self.assertNotIn("execute_retract_home", backend.calls)

    def test_confirm_failure_marks_uncertain_and_refuses_retraction(self):
        """release_hand succeeds but confirmation does not establish the outcome."""
        class OddBackend(FakeBackend):
            def release_hand(self, ctx):
                self.calls.append("release_hand")
                self.motion.append("release_hand")
                ctx.release_outcome = "unknown"     # accepted, outcome unclear
                return seq.StepResult(True, "sent", motion_issued=True)

        backend = OddBackend(pull_after_sleeps=2)
        result, ctx = deliver(backend, held_state())
        self.assertFalse(result.success)
        self.assertEqual(result.final_step, "confirm_released")
        self.assertEqual(ctx.handover.object_state, hs.UNCERTAIN)
        self.assertNotIn("execute_retract_home", backend.calls)

    def test_retraction_is_refused_while_uncertain_even_if_reached(self):
        state = held_state()
        state.mark_awaiting_pull(1000.0, hs.new_session_token())
        state.mark_release_uncertain("test")
        allowed, reason = state.retraction_allowed()
        self.assertFalse(allowed)
        self.assertIn("UNCERTAIN", reason)

    def test_a_later_delivery_is_refused_while_uncertain(self):
        state = held_state()
        state.mark_awaiting_pull(1000.0, hs.new_session_token())
        state.mark_release_uncertain("test")
        result, _ = deliver(FakeBackend(pull_after_sleeps=2), state)
        self.assertFalse(result.success)
        self.assertIn("UNCERTAIN", result.message)


class ArmingFailuresInTheRoute(unittest.TestCase):
    def test_an_echo_mismatch_fails_the_route_and_releases_nothing(self):
        backend = FakeBackend(fail={"arm_monitor_echo"}, pull_after_sleeps=2)
        result, ctx = deliver(backend, held_state())
        self.assertFalse(result.success)
        self.assertEqual(result.final_step, "arm_monitor")
        self.assertNotIn("release_hand", backend.calls)
        self.assertEqual(ctx.handover.object_state, hs.HELD)
        self.assertEqual(ctx.handover.snapshot().session_token, "",
                         "abandonment must clear the token")

    def test_an_arm_refusal_fails_the_route_and_releases_nothing(self):
        backend = FakeBackend(fail={"arm_monitor"}, pull_after_sleeps=2)
        result, ctx = deliver(backend, held_state())
        self.assertFalse(result.success)
        self.assertNotIn("release_hand", backend.calls)
        self.assertEqual(ctx.handover.object_state, hs.HELD)

    def test_the_monitor_is_disarmed_after_an_arming_failure(self):
        backend = FakeBackend(fail={"arm_monitor_echo"}, pull_after_sleeps=2)
        deliver(backend, held_state())
        self.assertIn("disarm_monitor_cleanup", backend.cleanup)


class StaleAndDuplicateRequests(unittest.TestCase):
    def test_a_duplicate_event_causes_exactly_one_release(self):
        class Doubling(FakeBackend):
            def sleep(self, seconds):
                FakeBackend.sleep(self, seconds)
                if self._sleeps == 3:               # try again after the first
                    self._inject_pull()

        backend = Doubling(pull_after_sleeps=2, exposes_settling=True)
        result, _ = deliver(backend, held_state())
        self.assertTrue(result.success, result.message)
        self.assertEqual(backend.calls.count("release_hand"), 1)

    def test_a_second_request_in_the_same_session_is_refused(self):
        state = held_state()
        token = hs.new_session_token()
        state.mark_awaiting_pull(1000.0, token)
        self.assertTrue(state.request_release(1000.1, "a", event_token=token).ok)
        second = state.request_release(1000.2, "b", event_token=token)
        self.assertFalse(second.ok)
        self.assertIn("duplicate", second.reason)

    def test_a_token_from_an_earlier_handover_is_refused(self):
        """The hole token identity closes: a late event cannot land in a new handover."""
        state = held_state()
        old_token = hs.new_session_token()
        state.mark_awaiting_pull(1000.0, old_token)
        state.abandon("timeout")
        state.begin("right", "cup", release_authorized=True)
        new_token = hs.new_session_token()
        state.mark_awaiting_pull(2000.0, new_token)
        self.assertNotEqual(old_token, new_token)
        stale = state.request_release(2000.1, "late", event_token=old_token)
        self.assertFalse(stale.ok)
        self.assertIn("stale", stale.reason)

    def test_a_stale_token_mid_delivery_produces_no_release(self):
        backend = FakeBackend(pull_after_sleeps=2, token_override="deadbeefdead")
        result, _ = deliver(backend, held_state())
        self.assertFalse(result.success)
        self.assertIn("TIMEOUT", result.message)
        self.assertNotIn("release_hand", backend.calls)

    def test_a_tokenless_request_is_refused_by_default(self):
        state = held_state()
        state.mark_awaiting_pull(1000.0, hs.new_session_token())
        v = state.request_release(1000.1, "monitor", event_token="")
        self.assertFalse(v.ok)
        self.assertIn("origin handover cannot be established", v.reason)
        self.assertIn("Receipt time cannot substitute", v.reason)

    def test_a_tokenless_request_is_flagged_when_the_policy_allows_it(self):
        state = hs.HandoverState(require_event_token=False)
        state.begin("right", "bottle", release_authorized=True)
        state.mark_awaiting_pull(1000.0, hs.new_session_token())
        v = state.request_release(1000.1, "monitor", event_token="")
        self.assertTrue(v.ok)
        self.assertTrue(v.origin_unverified, "an unidentifiable event must be flagged")


class ResumePrerequisites(unittest.TestCase):
    def test_resume_cannot_invent_a_held_object(self):
        backend = FakeBackend(pull_after_sleeps=2)
        result, _ = deliver(backend, hs.HandoverState(), start_step="arm_monitor")
        self.assertFalse(result.success)
        self.assertNotIn("release_hand", backend.calls)

    def test_resume_at_release_without_a_consumed_event_cannot_open_the_hand(self):
        state = held_state()
        backend = FakeBackend()
        result, _ = deliver(backend, state, start_step="release_hand")
        self.assertFalse(result.success, "resuming straight into release must not succeed")
        self.assertIn("release_hand_refused", backend.calls)
        self.assertNotIn("release_hand", backend.calls)
        self.assertIn("no validated pull has been consumed", result.message)
        self.assertEqual(state.object_state, hs.HELD, "the object must still be held")

    def test_resume_skips_only_the_earlier_steps(self):
        state = held_state()
        state.mark_awaiting_pull(1000.0, hs.new_session_token())
        backend = FakeBackend()
        result, _ = deliver(backend, state, start_step="disarm_monitor")
        self.assertEqual(result.steps_skipped[:3],
                         ["require_held_object", "await_localization", "enable_motion"])
        self.assertNotIn("arm_monitor", backend.calls)

    def test_an_unknown_start_step_runs_nothing(self):
        backend = FakeBackend()
        result, _ = deliver(backend, held_state(), start_step="nope")
        self.assertFalse(result.success)
        self.assertEqual(backend.calls, [])


class DryRunHasNoHardware(unittest.TestCase):
    def test_dry_run_traverses_the_whole_delivery_without_motion(self):
        backend = seq.DryRunBackend()
        ctx = seq.RouteContext(route="deliver", target="person", side="right",
                               dry_run=True, handover=hs.HandoverState())
        result = seq.run_route(r.DELIVER, backend, ctx, "", blockers=[])
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.steps_completed, EXPECTED_ORDER)
        self.assertFalse(result.motion_issued)

    def test_dry_run_writes_no_task_state(self):
        state = hs.HandoverState()
        backend = seq.DryRunBackend()
        ctx = seq.RouteContext(route="deliver", target="person", side="right",
                               dry_run=True, handover=state)
        seq.run_route(r.DELIVER, backend, ctx, "", blockers=[])
        self.assertEqual(state.object_state, hs.NONE)
        self.assertFalse(state.holding)

    def test_dry_run_backend_holds_no_client_objects(self):
        backend = seq.DryRunBackend()
        ctx = seq.RouteContext(route="deliver", side="right", dry_run=True,
                               handover=hs.HandoverState())
        seq.run_route(r.DELIVER, backend, ctx, "", blockers=[])
        for name in vars(backend):
            self.assertIn(name, ("calls", "cleanup", "_t"), "unexpected state %r" % name)

    def test_dry_run_does_not_wait_for_a_pull(self):
        backend = seq.DryRunBackend()
        ctx = seq.RouteContext(route="deliver", side="right", dry_run=True,
                               handover=hs.HandoverState())
        seq.run_route(r.DELIVER, backend, ctx, "", blockers=[])
        self.assertIn("await_pull", backend.calls)

    def test_blockers_stop_the_delivery_before_any_step(self):
        backend = FakeBackend(pull_after_sleeps=2)
        ctx = seq.RouteContext(route="deliver", side="right", dry_run=False,
                               handover=held_state())
        result = seq.run_route(r.DELIVER, backend, ctx, "",
                               blockers=["frame_contract: unresolved"])
        self.assertFalse(result.success)
        self.assertEqual(backend.calls, [])
        self.assertIn("NOT_READY", result.message)


if __name__ == "__main__":
    unittest.main()
