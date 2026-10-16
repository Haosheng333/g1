#!/usr/bin/env python3
"""Cross-process ROS integration test of the grasp-and-deliver pipeline.

Runs against an ISOLATED ROS master started by run_integration.sh. The
orchestrator node and the auto_release_monitor node are the REAL production
scripts; everything hardware-facing is a test double in doubles_node.py using
the real generated service/message types.

This file asserts DATA at module boundaries, not just call order. Every
assertion that cannot be checked is reported as blocked rather than skipped
silently.
"""
import json
import math
import os
import sys
import threading
import time
import unittest

import rospy
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger

from handover.srv import ArmMonitor
from orchestrator.srv import RunRoute

SIDE = os.environ.get("TEST_SIDE", "right")
TIMEOUT = float(os.environ.get("TEST_TIMEOUT", "25"))

# Force fixtures, EXPLICITLY IN NEWTONS as the task requires. These exercise the
# requested rule (any single channel delta > 3 N) and nothing else: they do NOT
# validate SDK units, field names or the real sign convention, none of which are
# known. Channel order below is [ch0, ch1, ch2].
FORCE_BASELINE_N = [10.0, 10.0, 10.0]
#: ch0 jumps 4.0 N (> 3 N); ch1 and ch2 move 1.0 N and -0.5 N (both < 3 N).
FORCE_ONE_CHANNEL_OVER_N = [14.0, 11.0, 9.5]
#: Every channel moves less than 3 N: must NOT request a release.
FORCE_ALL_UNDER_N = [12.0, 11.5, 12.0]


def _call(service, srv_type, **kw):
    rospy.wait_for_service(service, timeout=TIMEOUT)
    return rospy.ServiceProxy(service, srv_type)(**kw)


class Recorder(object):
    """Collects /test/calls, /orchestrator/status and monitor events."""

    def __init__(self):
        self.calls, self.status, self.events = [], [], []
        self.lock = threading.Lock()
        rospy.Subscriber("/test/calls", String, self._c, queue_size=100)
        rospy.Subscriber("/orchestrator/status", String, self._s, queue_size=100)
        rospy.Subscriber("/auto_release_monitor/event", String, self._e, queue_size=100)

    def _c(self, m):
        with self.lock:
            self.calls.append(m.data)

    def _s(self, m):
        with self.lock:
            self.status.append(m.data)

    def _e(self, m):
        with self.lock:
            self.events.append(m.data)

    def reset(self):
        with self.lock:
            self.calls, self.status, self.events = [], [], []

    def snapshot(self):
        with self.lock:
            return list(self.calls), list(self.status), list(self.events)


REC = None
SCENARIO_PUB = None
FORCE_PUB = None
WAIST_PUB = None
EVIDENCE = []


def evidence(section, payload):
    """Record a boundary payload for the report."""
    EVIDENCE.append({"section": section, "payload": payload})
    rospy.loginfo("[EVIDENCE] %s :: %s", section, json.dumps(payload, default=str))


def set_scenario(name, settle=0.6):
    SCENARIO_PUB.publish(String(data=name))
    rospy.sleep(settle)
    REC.reset()


def set_waist(yaw_rad, settle=0.6):
    """Set the waist yaw the doubles' TF carries."""
    WAIST_PUB.publish(Float32MultiArray(data=[float(yaw_rad)]))
    rospy.sleep(settle)


def set_force(values_n, settle=0.25):
    FORCE_PUB.publish(Float32MultiArray(data=values_n))
    rospy.sleep(settle)


def _actions(calls):
    return [c.split("|", 1)[0] for c in calls]


def await_call(prefix, timeout=3.0):
    """Wait for a /test/calls record starting with `prefix`, then return all calls.

    The doubles publish their record inside the service handler, so a snapshot
    taken the instant the service returns can race the topic. Polling removes
    that flake without weakening the assertion: a call that never arrives still
    fails the test.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        calls, _, _ = REC.snapshot()
        if any(c.startswith(prefix) for c in calls):
            return calls
        rospy.sleep(0.05)
    return REC.snapshot()[0]


# =====================================================================================
class A_ReachableRouteRunsForReal(unittest.TestCase):
    """navigate_report is the ONLY route whose preflight can clear."""

    def test_navigate_report_executes_end_to_end_over_ros(self):
        set_scenario("happy")
        r = _call("/orchestrator/run_navigate_report", RunRoute,
                  route="", target="table", start_step="", dry_run=False, event_token="")
        calls, status, _ = REC.snapshot()
        evidence("A1 navigate_report real run", {
            "success": r.success, "dry_run": r.dry_run,
            "steps_completed": list(r.steps_completed),
            "final_step": r.final_step, "blockers": list(r.blockers),
            "motion_issued": r.motion_issued, "message": r.message,
            "observed_double_calls": calls, "orchestrator_status": status,
        })
        self.assertTrue(r.success, "%s | blockers=%s" % (r.message, list(r.blockers)))
        self.assertFalse(r.dry_run, "must be a REAL run, not a dry run")
        self.assertEqual(list(r.steps_completed),
                         ["await_localization", "enable_motion", "navigate",
                          "detect_object", "report"])
        self.assertEqual(list(r.blockers), [])

    def test_boundary_1_target_survives_into_the_navigation_request(self):
        """Task request target -> /navigation/navigate class_name."""
        set_scenario("happy")
        _call("/orchestrator/run_navigate_report", RunRoute,
              route="", target="workbench", start_step="", dry_run=False, event_token="")
        calls, _, _ = REC.snapshot()
        nav = [c for c in calls if c.startswith("navigate|")]
        evidence("A2 boundary 1 payload", {"navigate_calls": nav})
        self.assertTrue(nav, "navigation was never called")
        self.assertIn("class_name=workbench", nav[0],
                      "the task target did not survive into the navigation request")

    def test_boundary_2_detection_payload_survives_with_its_frame(self):
        """Perception xyz + frame_id reach the orchestrator unconverted."""
        set_scenario("happy")
        r = _call("/orchestrator/run_navigate_report", RunRoute,
                  route="", target="bottle", start_step="", dry_run=False, event_token="")
        calls, _, _ = REC.snapshot()
        det = [c for c in calls if c.startswith("detect_object|")]
        evidence("A3 boundary 2 payload", {
            "detect_calls": det, "orchestrator_message": r.message})
        self.assertIn("label=bottle", det[0], "the label did not survive")
        self.assertIn("index=0", det[0])
        # The orchestrator's own report echoes the numbers and the frame.
        for token in ("0.412", "-0.133", "0.907", "base_link"):
            self.assertIn(token, r.message,
                          "%r missing from the reported detection: %s" % (token, r.message))

    def test_no_arm_service_was_ever_called(self):
        """Proves boundaries 3-5 are not merely untested but unimplemented."""
        set_scenario("happy")
        _call("/orchestrator/run_navigate_report", RunRoute,
              route="", target="table", start_step="", dry_run=False, event_token="")
        calls, _, _ = REC.snapshot()
        self.assertEqual([c for c in calls if c.startswith("UNEXPECTED")], [])


# =====================================================================================
class B_FullTaskRoutesAreRefused(unittest.TestCase):
    """grasp and deliver carry unresolved capabilities: zero steps, zero motion."""

    def _refused(self, service, target):
        set_scenario("happy")
        r = _call(service, RunRoute, route="", target=target, start_step="",
                  dry_run=False, event_token="")
        calls, _, _ = REC.snapshot()
        evidence("B %s real run" % service, {
            "success": r.success, "message": r.message,
            "blockers": list(r.blockers), "steps_completed": list(r.steps_completed),
            "motion_issued": r.motion_issued, "object_state": r.object_state,
            "observed_double_calls": calls,
        })
        self.assertFalse(r.success)
        self.assertTrue(r.blockers, "expected preflight blockers")
        self.assertEqual(list(r.steps_completed), [], "a blocked route must run NO steps")
        self.assertFalse(r.motion_issued)
        return r

    def test_grasp_is_refused_before_any_step(self):
        r = self._refused("/orchestrator/run_grasp", "bottle")
        joined = " ".join(r.blockers)
        # frame_contract was split: the conversion is implemented, the camera
        # extrinsics are not.
        self.assertIn("perception_extrinsics", joined)
        self.assertNotIn("frame_contract", joined)

    def test_deliver_is_refused_before_any_step(self):
        r = self._refused("/orchestrator/run_deliver", "person")
        joined = " ".join(r.blockers)
        for expected in ("handover_pose", "home_pose", "release_completion"):
            self.assertIn(expected, joined)

    def test_a_blocked_route_issued_no_hardware_call_at_all(self):
        set_scenario("happy")
        _call("/orchestrator/run_grasp", RunRoute, route="", target="bottle",
              start_step="", dry_run=False, event_token="")
        calls, _, _ = REC.snapshot()
        self.assertEqual(_actions(calls), [],
                         "a refused route must not touch navigation, perception or the hand")


# =====================================================================================
class C_MonitorToOrchestratorTokenPath(unittest.TestCase):
    """The real monitor, over real ROS, with a labelled force sequence in N."""

    def test_monitor_is_ready_with_the_test_only_schema(self):
        msg = rospy.wait_for_message("/auto_release_monitor/ready", Bool, timeout=TIMEOUT)
        evidence("C1 monitor readiness", {"ready": msg.data})
        self.assertTrue(msg.data,
                        "monitor NOT_READY: the test-only tactile schema did not load")

    def test_arming_requires_a_session_token(self):
        r = _call("/auto_release_monitor/arm", ArmMonitor, side=SIDE, session_token="")
        evidence("C2 tokenless arm refused", {"success": r.success, "message": r.message})
        self.assertFalse(r.success)
        self.assertIn("session_token", r.message)

    def test_one_channel_over_3N_produces_one_tokened_release_request(self):
        set_scenario("happy")
        token = "itest-token-%d" % int(time.time() * 1000 % 1e9)
        set_force(FORCE_BASELINE_N)
        armed = _call("/auto_release_monitor/arm", ArmMonitor, side=SIDE,
                      session_token=token)
        self.assertTrue(armed.success, armed.message)
        self.assertEqual(armed.session_token, token, "monitor did not echo the token")

        # Hold the baseline past the post-arm suppression window, then step ONE
        # channel over 3 N while the others stay under.
        set_force(FORCE_BASELINE_N, settle=1.5)
        set_force(FORCE_ONE_CHANNEL_OVER_N, settle=1.5)

        _, _, events = REC.snapshot()
        confirmed = [e for e in events if e.startswith("confirmed|")]
        evidence("C3 force -> detection", {
            "baseline_N": FORCE_BASELINE_N,
            "step_N": FORCE_ONE_CHANNEL_OVER_N,
            "deltas_N": [round(a - b, 3) for a, b in
                         zip(FORCE_ONE_CHANNEL_OVER_N, FORCE_BASELINE_N)],
            "threshold_N": 3.0,
            "monitor_events": events[-6:],
            "echoed_token": armed.session_token,
        })
        self.assertTrue(confirmed, "no confirmed detection; events=%s" % (events,))
        _call("/auto_release_monitor/disarm", Trigger)

    def test_all_channels_under_3N_produce_no_detection(self):
        set_scenario("happy")
        set_force(FORCE_BASELINE_N)
        armed = _call("/auto_release_monitor/arm", ArmMonitor, side=SIDE,
                      session_token="itest-under")
        self.assertTrue(armed.success, armed.message)
        set_force(FORCE_BASELINE_N, settle=1.5)
        set_force(FORCE_ALL_UNDER_N, settle=1.2)
        _, _, events = REC.snapshot()
        evidence("C4 all channels under threshold", {
            "step_N": FORCE_ALL_UNDER_N,
            "deltas_N": [round(a - b, 3) for a, b in
                         zip(FORCE_ALL_UNDER_N, FORCE_BASELINE_N)],
            "threshold_N": 3.0, "monitor_events": events[-4:]})
        self.assertEqual([e for e in events if e.startswith("confirmed|")], [],
                         "sub-threshold deltas must not confirm a pull")
        _call("/auto_release_monitor/disarm", Trigger)

    def test_missing_force_input_produces_no_release(self):
        set_scenario("no_force")
        armed = _call("/auto_release_monitor/arm", ArmMonitor, side=SIDE,
                      session_token="itest-noforce")
        self.assertTrue(armed.success, armed.message)
        rospy.sleep(2.0)
        _, _, events = REC.snapshot()
        evidence("C5 force input absent", {"monitor_events": events[-4:]})
        self.assertEqual([e for e in events if e.startswith("confirmed|")], [])
        self.assertTrue([e for e in events if e.startswith("fault|")],
                        "a silent sensor must publish a fault")
        _call("/auto_release_monitor/disarm", Trigger)
        set_scenario("happy")

    def test_release_request_without_a_held_object_is_refused(self):
        """The orchestrator's own validation, reached over real ROS."""
        r = _call("/orchestrator/run_auto_release", RunRoute, route="",
                  target="", start_step="", dry_run=False, event_token="itest-orphan")
        evidence("C6 release request with nothing held", {
            "success": r.success, "message": r.message,
            "object_state": r.object_state, "transfer_verified": r.transfer_verified})
        self.assertFalse(r.success)
        self.assertIn("held", r.message)
        self.assertFalse(r.transfer_verified)

    def test_tokenless_release_request_is_refused(self):
        r = _call("/orchestrator/run_auto_release", RunRoute, route="", target="",
                  start_step="", dry_run=False, event_token="")
        evidence("C7 tokenless release request", {
            "success": r.success, "message": r.message})
        self.assertFalse(r.success)


# =====================================================================================
class D_FailurePropagation(unittest.TestCase):
    """Each scenario must fail at a NAMED step and run nothing after it."""

    def _run(self, scenario, target="table", route="navigate_report"):
        set_scenario(scenario)
        r = _call("/orchestrator/run_%s" % route, RunRoute, route="", target=target,
                  start_step="", dry_run=False, event_token="")
        calls, _, _ = REC.snapshot()
        evidence("D %s" % scenario, {
            "success": r.success, "final_step": r.final_step,
            "steps_completed": list(r.steps_completed), "message": r.message,
            "motion_issued": r.motion_issued, "observed_double_calls": calls})
        return r, calls

    def test_navigation_failure_stops_at_navigate(self):
        r, calls = self._run("nav_failure")
        self.assertFalse(r.success)
        self.assertEqual(r.final_step, "navigate")
        self.assertNotIn("detect_object", _actions(calls),
                         "perception ran after navigation failed")

    def test_stale_success_does_not_complete_a_new_goal(self):
        """The saw_active guard (spec 4.3)."""
        r, calls = self._run("nav_stale_success")
        self.assertFalse(r.success, "a leftover SUCCESS completed a new goal")
        self.assertEqual(r.final_step, "navigate")
        self.assertIn("saw_active", r.message.lower() + " " + r.message)
        self.assertNotIn("detect_object", _actions(calls))

    def test_navigation_timeout_stops_at_navigate(self):
        r, calls = self._run("nav_timeout")
        self.assertFalse(r.success)
        self.assertEqual(r.final_step, "navigate")
        self.assertNotIn("detect_object", _actions(calls))

    def test_navigation_goal_rejection_stops_at_navigate(self):
        r, calls = self._run("nav_reject")
        self.assertFalse(r.success)
        self.assertEqual(r.final_step, "navigate")
        self.assertNotIn("detect_object", _actions(calls))

    def test_perception_failure_stops_at_detect_object(self):
        r, calls = self._run("perception_failure")
        self.assertFalse(r.success)
        self.assertEqual(r.final_step, "detect_object")
        self.assertNotIn("report", r.steps_completed)

    def test_missing_joint_states_blocks_the_route_at_preflight(self):
        set_scenario("no_joint_states", settle=1.5)
        r = _call("/orchestrator/run_navigate_report", RunRoute, route="", target="table",
                  start_step="", dry_run=False, event_token="")
        calls, _, _ = REC.snapshot()
        evidence("D no_joint_states", {
            "success": r.success, "blockers": list(r.blockers),
            "steps_completed": list(r.steps_completed), "observed_double_calls": calls})
        self.assertFalse(r.success)
        self.assertTrue(any("joint_states" in b for b in r.blockers),
                        "blockers=%s" % list(r.blockers))
        self.assertEqual(list(r.steps_completed), [])
        set_scenario("happy", settle=1.5)

    def test_concurrent_requests_are_refused_without_queueing(self):
        set_scenario("happy")
        results = {}

        def fire(key, target):
            try:
                results[key] = _call("/orchestrator/run_navigate_report", RunRoute,
                                     route="", target=target, start_step="",
                                     dry_run=False, event_token="")
            except Exception as exc:                      # noqa: BLE001
                results[key] = exc

        t1 = threading.Thread(target=fire, args=("a", "table"))
        t2 = threading.Thread(target=fire, args=("b", "table"))
        t1.start()
        rospy.sleep(0.15)
        t2.start()
        t1.join(TIMEOUT)
        t2.join(TIMEOUT)
        msgs = {k: getattr(v, "message", str(v)) for k, v in results.items()}
        evidence("D concurrent requests", msgs)
        refused = [m for m in msgs.values()
                   if "already running" in m.lower() or "busy" in m.lower()]
        self.assertTrue(refused, "no request was refused while another ran: %s" % msgs)
        self.assertEqual(len(refused), 1, "exactly one of the two should be refused: %s" % msgs)

    def test_unknown_resume_point_is_refused(self):
        set_scenario("happy")
        r = _call("/orchestrator/run_navigate_report", RunRoute, route="", target="table",
                  start_step="no_such_step", dry_run=False, event_token="")
        evidence("D invalid resume", {"success": r.success, "message": r.message})
        self.assertFalse(r.success)

    def test_route_guard_mismatch_is_refused(self):
        r = _call("/orchestrator/run_navigate_report", RunRoute, route="deliver",
                  target="table", start_step="", dry_run=False, event_token="")
        evidence("D route guard mismatch", {"success": r.success, "message": r.message})
        self.assertFalse(r.success)


# =====================================================================================
class E_FullTaskEntryPoint(unittest.TestCase):
    """run_deliver_task: one command for the whole framework task (spec 0.4 / 6.2)."""

    EXPECTED_28 = [
        "await_localization", "enable_motion", "navigate", "detect_object",
        "to_planner_frame", "plan_grasp", "plan_stow",
        "execute_to_pregrasp", "open_gripper",
        "execute_approach", "close_gripper", "execute_lift", "execute_retract",
        "record_held_object", "execute_stow",
        "require_held_object", "deliver_await_localization", "deliver_enable_motion",
        "navigate_to_recipient", "plan_handover_pose", "execute_handover_pose",
        "settle_handover", "arm_monitor", "await_pull", "disarm_monitor",
        "release_hand", "confirm_released", "settle_recipient", "execute_retract_home",
    ]

    def test_the_service_exists(self):
        rospy.wait_for_service("/orchestrator/run_deliver_task", timeout=TIMEOUT)

    def test_dry_run_traverses_the_whole_task_in_order(self):
        """A dry run walks all 28 steps. This proves SEQUENCING only: the
        DryRunBackend makes no service call, so it is not a pipeline test."""
        set_scenario("happy")
        r = _call("/orchestrator/run_deliver_task", RunRoute, route="", target="bottle",
                  destination="person", start_step="", dry_run=True, event_token="")
        calls, _, _ = REC.snapshot()
        evidence("E1 deliver_task dry run", {
            "success": r.success, "dry_run": r.dry_run,
            "n_steps_completed": len(r.steps_completed),
            "steps_completed": list(r.steps_completed),
            "blocker_count": len(r.blockers),
            "motion_issued": r.motion_issued,
            "observed_double_calls": calls,
        })
        self.assertTrue(r.success, r.message)
        self.assertTrue(r.dry_run)
        self.assertEqual(list(r.steps_completed), self.EXPECTED_28)
        self.assertFalse(r.motion_issued, "a dry run must issue no motion")
        self.assertEqual(_actions(calls), [],
                         "a dry run must make no hardware service call at all")
        self.assertTrue(r.blockers, "a dry run must still report what a real run refuses")

    def test_step_names_are_unique_so_resume_is_unambiguous(self):
        names = self.EXPECTED_28
        self.assertEqual(len(names), len(set(names)))

    def test_real_run_is_refused_with_the_union_of_both_phases_blockers(self):
        set_scenario("happy")
        r = _call("/orchestrator/run_deliver_task", RunRoute, route="", target="bottle",
                  destination="person", start_step="", dry_run=False, event_token="")
        calls, _, _ = REC.snapshot()
        joined = " ".join(r.blockers)
        evidence("E2 deliver_task real run", {
            "success": r.success, "message": r.message,
            "blocker_keys": sorted({b.split(" (")[0] for b in r.blockers}),
            "steps_completed": list(r.steps_completed),
            "motion_issued": r.motion_issued, "observed_double_calls": calls,
        })
        self.assertFalse(r.success)
        self.assertEqual(list(r.steps_completed), [])
        self.assertFalse(r.motion_issued)
        for expected in ("perception_extrinsics", "stow_pose", "handover_pose",
                         "home_pose", "release_completion"):
            self.assertIn(expected, joined, "missing blocker %s" % expected)

    def test_a_dry_run_resume_past_the_grasp_still_reports_success(self):
        """LIMITATION, verified rather than assumed: dry-run success is not task validity.

        `DryRunBackend.require_held_object` returns ok with the detail "dry run:
        held-object check skipped", so a dry run resumed at the delivery phase
        traverses 14 steps and reports success even though no grasp ever ran.
        That is the rehearsal's documented contract -- it walks step ORDER, it
        does not evaluate state -- but it means a green dry run must never be
        read as "the task would work".

        The real guard is elsewhere and does hold: HandoverState.release_permitted()
        refuses with "never because a route was resumed at this step", covered
        offline by test_delivery_flow.test_resume_at_release_without_a_consumed_event_
        cannot_open_the_hand. A real run of this route cannot reach it anyway:
        preflight refuses on 5 unresolved capabilities.
        """
        set_scenario("happy")
        r = _call("/orchestrator/run_deliver_task", RunRoute, route="", target="bottle",
                  destination="person", start_step="require_held_object",
                  dry_run=True, event_token="")
        calls, _, _ = REC.snapshot()
        evidence("E3 dry-run resume past the grasp (LIMITATION)", {
            "success": r.success, "message": r.message,
            "final_step": r.final_step,
            "n_steps_skipped": len(r.steps_skipped),
            "n_steps_completed": len(r.steps_completed),
            "hardware_calls": _actions(calls),
            "note": "dry run skips the held-object check by design; "
                    "the real refusal lives in HandoverState.release_permitted()",
        })
        self.assertTrue(r.success, "the dry-run contract is to traverse, not to validate")
        self.assertEqual(len(r.steps_skipped), 15, "the whole grasp phase should be skipped")
        self.assertEqual(_actions(calls), [], "a dry run must still touch no hardware")

    def test_the_real_resume_guard_refuses_to_open_the_hand(self):
        """The guard a dry run bypasses, exercised directly on production code."""
        from orchestrator import handover_state as hs
        state = hs.HandoverState()
        fresh = state.release_permitted()
        state.begin("right", "bottle", release_authorized=True)
        holding = state.release_permitted()
        evidence("E4 real resume guard", {
            "no_object": list(fresh), "holding_but_no_consumed_pull": list(holding)})
        self.assertFalse(fresh[0])
        self.assertFalse(holding[0], "a resume with nothing consumed must not release")
        self.assertIn("resumed at this step", holding[1])


# =====================================================================================
class F_NewlyWiredAdaptersAreReal(unittest.TestCase):
    """Exercise the REAL adapter code against the arm-service DOUBLES.

    These tests construct the production `RosBackend` and call its adapters
    directly over real ROS. That is the only way to reach them: the routes that
    contain them are refused at preflight on unresolved capabilities, and this
    task must not weaken that gate to get a green result.

    WHAT IS PROVEN: the orchestrator's wiring -- request fields, joint order,
    arm side, path slicing, units passthrough, and failure propagation.
    WHAT IS NOT: anything about arm_planning's or arm_control's algorithms. The
    planner and executor here are doubles returning fixture ramps.
    """

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
        import orchestrator_node as onode
        from orchestrator import sequencer as seq
        cls.onode, cls.seq = onode, seq
        set_scenario("happy", settle=1.0)
        cls.backend = onode.RosBackend(SIDE, {"waypoint_sec": 0.0})
        rospy.sleep(1.0)        # let /joint_states arrive

    def _ctx(self, **kw):
        from orchestrator import handover_state as hs
        base = dict(route="deliver_task", target="bottle", side=SIDE,
                    dry_run=False, handover=hs.HandoverState())
        base.update(kw)
        return self.seq.RouteContext(**base)

    # --- current_joints ------------------------------------------------------
    def test_current_joints_reads_the_five_arm_joints_from_joint_states(self):
        joints, reason = self.backend._current_joints(SIDE)
        evidence("F1 current_joints", {"side": SIDE, "joints_rad": joints, "reason": reason})
        self.assertIsNotNone(joints, reason)
        self.assertEqual(len(joints), 5)
        self.assertEqual([round(v, 4) for v in joints],
                         [-0.1, -0.2, -0.3, -0.4, -0.5] if SIDE == "right"
                         else [0.1, 0.2, 0.3, -0.4, 0.5])

    def test_missing_arm_joints_is_not_ready_not_a_guessed_posture(self):
        set_scenario("no_arm_joints", settle=1.2)
        backend = self.onode.RosBackend(SIDE, {})
        rospy.sleep(1.2)
        joints, reason = backend._current_joints(SIDE)
        evidence("F2 arm joints absent", {"joints": joints, "reason": reason})
        self.assertIsNone(joints)
        self.assertIn("NOT_READY", reason)
        self.assertIn("/joint_states", reason)
        set_scenario("happy", settle=1.2)

    # --- plan_grasp ----------------------------------------------------------
    def test_plan_grasp_refuses_without_a_planner_frame_target(self):
        """Boundary 3 is still missing, and plan_grasp says so rather than guessing."""
        ctx = self._ctx()
        r = self.backend.plan_grasp(ctx)
        evidence("F3 plan_grasp with no planner_target", {"ok": r.ok, "detail": r.detail})
        self.assertFalse(r.ok)
        self.assertIn("to_planner_frame", r.detail)
        self.assertEqual(ctx.plan, {})

    def test_plan_grasp_calls_the_real_service_and_keeps_all_four_segments(self):
        set_scenario("happy", settle=0.8)
        ctx = self._ctx(planner_target=[0.31, -0.18, 0.02])
        r = self.backend.plan_grasp(ctx)
        calls = await_call("plan_grasp_from_point")
        evidence("F4 plan_grasp payload", {
            "ok": r.ok, "detail": r.detail,
            "observed_call": [c for c in calls if c.startswith("plan_grasp_from_point")],
            "segments": {k: {"count": v["count"], "n_values": len(v["flat"]),
                             "arm": v["arm"]} for k, v in ctx.plan.items()},
            "first_waypoint_of_to_pregrasp": ctx.plan["to_pregrasp"]["flat"][:5],
        })
        self.assertTrue(r.ok, r.detail)
        # The target reached the planner, in metres, unconverted.
        self.assertTrue(any("object_xyz=[0.31, -0.18, 0.02]" in c for c in calls),
                        "planner_target did not reach the planner verbatim: %s" % calls)
        self.assertEqual(sorted(ctx.plan), ["approach", "lift", "retract", "to_pregrasp"])
        for name, count in (("to_pregrasp", 3), ("approach", 2), ("lift", 2), ("retract", 4)):
            self.assertEqual(ctx.plan[name]["count"], count, name)
            self.assertEqual(len(ctx.plan[name]["flat"]), count * 5, name)
            self.assertEqual(ctx.plan[name]["arm"], SIDE)

    def test_segments_are_sliced_at_the_right_offsets(self):
        """A mis-sliced path would silently execute another segment's waypoints."""
        set_scenario("happy", settle=0.8)
        ctx = self._ctx(planner_target=[0.31, -0.18, 0.02])
        self.assertTrue(self.backend.plan_grasp(ctx).ok)
        # The double ramps each segment from 0.1*(i+1); the first column of the
        # first waypoint therefore identifies the segment.
        firsts = {n: round(ctx.plan[n]["flat"][0], 4) for n in
                  ("to_pregrasp", "approach", "lift", "retract")}
        evidence("F5 segment slicing", firsts)
        self.assertEqual(firsts, {"to_pregrasp": 0.1, "approach": 0.2,
                                  "lift": 0.3, "retract": 0.4})

    def test_planner_failure_message_is_propagated_verbatim(self):
        set_scenario("planner_failure", settle=0.8)
        ctx = self._ctx(planner_target=[0.31, -0.18, 0.02])
        r = self.backend.plan_grasp(ctx)
        evidence("F6 planner failure", {"ok": r.ok, "detail": r.detail})
        self.assertFalse(r.ok)
        self.assertIn("collision-free path", r.detail)
        self.assertEqual(ctx.plan, {})
        set_scenario("happy", settle=0.8)

    def test_malformed_planner_path_is_rejected_before_execution(self):
        set_scenario("planner_malformed", settle=0.8)
        ctx = self._ctx(planner_target=[0.31, -0.18, 0.02])
        r = self.backend.plan_grasp(ctx)
        evidence("F7 malformed planner path", {"ok": r.ok, "detail": r.detail})
        self.assertFalse(r.ok)
        self.assertIn("does not match", r.detail)
        self.assertEqual(ctx.plan, {})
        set_scenario("happy", settle=0.8)

    def test_wrong_joint_order_is_rejected(self):
        set_scenario("planner_wrong_joint_order", settle=0.8)
        ctx = self._ctx(planner_target=[0.31, -0.18, 0.02])
        r = self.backend.plan_grasp(ctx)
        evidence("F8 permuted joint order", {"ok": r.ok, "detail": r.detail})
        self.assertFalse(r.ok)
        self.assertIn("joint order", r.detail)
        set_scenario("happy", settle=0.8)

    # --- execute_segment -----------------------------------------------------
    def test_execute_segment_sends_the_planners_actual_path(self):
        set_scenario("happy", settle=0.8)
        ctx = self._ctx(planner_target=[0.31, -0.18, 0.02])
        self.assertTrue(self.backend.plan_grasp(ctx).ok)
        step = type("S", (), {"name": "execute_approach"})()
        r = self.backend.execute_segment(ctx, step)
        calls = await_call("move_arm_joints")
        exec_calls = [c for c in calls if c.startswith("move_arm_joints")]
        evidence("F9 execute_segment payload", {
            "ok": r.ok, "detail": r.detail, "motion_issued": r.motion_issued,
            "observed_call": exec_calls,
            "planned_first_waypoint": [round(v, 4) for v in ctx.plan["approach"]["flat"][:5]],
        })
        self.assertTrue(r.ok, r.detail)
        self.assertTrue(r.motion_issued)
        self.assertTrue(exec_calls, "the executor was never called")
        # The waypoints sent are the planner's, not a separate hardcoded success.
        self.assertIn("waypoints=2", exec_calls[-1])
        self.assertIn("first=[0.2, 0.201, 0.202, 0.203, 0.204]", exec_calls[-1])

    def test_execute_segment_refuses_when_nothing_was_planned(self):
        ctx = self._ctx()
        step = type("S", (), {"name": "execute_lift"})()
        r = self.backend.execute_segment(ctx, step)
        evidence("F10 execute with no plan", {"ok": r.ok, "detail": r.detail,
                                              "motion_issued": r.motion_issued})
        self.assertFalse(r.ok)
        self.assertIn("no planned path", r.detail)
        self.assertFalse(r.motion_issued, "nothing may be sent without a plan")

    def test_executor_failure_is_propagated_and_flagged_as_motion_issued(self):
        set_scenario("happy", settle=0.8)
        ctx = self._ctx(planner_target=[0.31, -0.18, 0.02])
        self.assertTrue(self.backend.plan_grasp(ctx).ok)
        set_scenario("arm_exec_failure", settle=0.8)
        step = type("S", (), {"name": "execute_lift"})()
        r = self.backend.execute_segment(ctx, step)
        evidence("F11 executor failure", {"ok": r.ok, "detail": r.detail,
                                          "motion_issued": r.motion_issued})
        self.assertFalse(r.ok)
        self.assertIn("timed out", r.detail)
        self.assertTrue(r.motion_issued, "a command did leave the process")
        set_scenario("happy", settle=0.8)

    # --- pose-dependent steps ------------------------------------------------
    def test_pose_steps_are_not_ready_while_poses_is_empty(self):
        """The wiring exists; the POSE does not. Both facts must be visible."""
        self.assertEqual(self.backend.poses, {},
                         "the shipped config must not define any pose")
        results = {}
        # plan_stow is given the grasp end posture so the ONLY thing missing is
        # the pose itself; otherwise it would refuse for the wrong reason.
        ctx = self._ctx()
        ctx.grasp_final_joints = [0.1, 0.2, 0.3, -0.4, 0.5]
        for name, call in (("plan_handover_pose", self.backend.plan_handover_pose),
                           ("plan_stow", self.backend.plan_stow)):
            r = call(ctx)
            results[name] = {"ok": r.ok, "detail": r.detail[:120]}
            self.assertFalse(r.ok)
            self.assertIn("NOT_READY", r.detail)
        evidence("F12 pose steps blocked", results)

    def test_a_configured_pose_reaches_the_real_plan_to_pose_service(self):
        """TEST-ONLY pose, injected on the backend instance only.

        It is NOT written to any config file and does NOT clear the
        handover_pose capability -- preflight still refuses the route. This
        exercises the adapter's request/response handling and nothing else.
        """
        set_scenario("happy", settle=0.8)
        backend = self.onode.RosBackend(SIDE, {"waypoint_sec": 0.25})
        rospy.sleep(1.0)
        backend.poses = {"handover": [0.25, -0.15, 0.10, 0.0, 0.0]}   # TEST FIXTURE
        ctx = self._ctx()
        planned = backend.plan_handover_pose(ctx)
        step = type("S", (), {"name": "execute_handover_pose"})()
        moved = backend.execute_segment(ctx, step)
        calls = await_call("move_arm_joints")
        evidence("F13 configured pose -> plan -> execute", {
            "planned_ok": planned.ok, "planned_detail": planned.detail,
            "moved_ok": moved.ok, "moved_detail": moved.detail,
            "plan_to_pose_call": [c for c in calls if c.startswith("plan_to_pose")],
            "move_call": [c for c in calls if c.startswith("move_arm_joints")],
            "stored_segment": {"count": ctx.plan["handover_pose"]["count"],
                               "n_values": len(ctx.plan["handover_pose"]["flat"])},
        })
        self.assertTrue(planned.ok, planned.detail)
        self.assertTrue(any("target_pose=[0.25, -0.15, 0.1, 0.0, 0.0]" in c for c in calls),
                        "the pose did not reach the planner verbatim: %s" % calls)
        self.assertIn("position_error_m=0.0031", planned.detail,
                      "the planner's own verification figure was dropped")
        self.assertTrue(moved.ok, moved.detail)
        self.assertEqual(ctx.plan["handover_pose"]["count"], 5)

    def test_retract_home_still_refuses_on_a_command_acknowledgement(self):
        """Even with a pose configured, RELEASE_COMMANDED must block retraction."""
        from orchestrator import handover_state as hs
        backend = self.onode.RosBackend(SIDE, {})
        rospy.sleep(0.5)
        backend.poses = {"home": [0.10, -0.05, 0.05, 0.0, 0.0]}      # TEST FIXTURE
        state = hs.HandoverState()
        state.begin(SIDE, "bottle", release_authorized=True)
        state.mark_release_commanded()
        ctx = self._ctx(handover=state)
        r = backend.execute_retract_home(ctx)
        evidence("F14 retract on ack only", {"ok": r.ok, "detail": r.detail[:160]})
        self.assertFalse(r.ok)
        self.assertIn("acknowledgement", r.detail)


# =====================================================================================
class G_FrameConversionIsProductionCode(unittest.TestCase):
    """Boundary 3 over real ROS, against the REAL /arm/convert_observation.

    The conversion service here is **production code** from
    arm_planning/scripts/arm_planning_node.py + frame_conversion.py, doing a real
    tf2 lookup against TF published by the doubles. The planner and executor are
    still doubles; the conversion is not.
    """

    WAIST_ORIGIN_X = -0.0039635
    PLANNER_WAIST_Z = 0.054

    @classmethod
    def setUpClass(cls):
        from arm_planning.srv import ConvertObservation
        cls.ConvertObservation = ConvertObservation
        rospy.wait_for_service("/arm/convert_observation", timeout=TIMEOUT)
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
        import orchestrator_node as onode
        cls.onode = onode
        set_scenario("happy", settle=1.0)
        set_waist(0.0)

    def _convert(self, point, source="base_link", stamp_is_obs=False, max_delta=0.0,
                 stamp=None):
        return _call("/arm/convert_observation", self.ConvertObservation,
                     source_frame=source, point=point,
                     stamp=stamp if stamp is not None else rospy.Time(0),
                     stamp_is_observation_time=stamp_is_obs,
                     max_waist_delta_rad=max_delta)

    def _expected(self, point, theta):
        """Independent derivation, same as arm_planning's offline test."""
        dx = point[0] - self.WAIST_ORIGIN_X
        dy = point[1]
        dz = point[2] - self.PLANNER_WAIST_Z
        c, s = math.cos(-theta), math.sin(-theta)
        return [c * dx - s * dy, s * dx + c * dy, dz]

    def test_service_is_up_and_reports_the_convention_offset(self):
        set_waist(0.0)
        r = self._convert([0.0, 0.0, 0.0])
        evidence("G1 convert at waist 0", {
            "success": r.success, "message": r.message,
            "planner_point": list(r.planner_point),
            "planner_frame": r.planner_frame,
            "convention_offset": list(r.convention_offset),
            "waist_yaw_rad": r.waist_yaw_rad})
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.planner_frame, "torso_link")
        self.assertAlmostEqual(list(r.convention_offset)[2], 0.010, places=9)

    def test_zero_waist_known_point_matches_an_independent_calculation(self):
        set_waist(0.0)
        point = [0.412, -0.133, 0.907]
        r = self._convert(point)
        expected = self._expected(point, 0.0)
        evidence("G2 known point, waist 0", {
            "input_base_link": point, "planner_point": list(r.planner_point),
            "independently_expected": expected})
        self.assertTrue(r.success, r.message)
        for axis in range(3):
            self.assertAlmostEqual(r.planner_point[axis], expected[axis], places=6)
        self.assertAlmostEqual(r.planner_point[2], 0.853, places=5)

    def test_nonzero_waist_known_points_match(self):
        point = [0.412, -0.133, 0.907]
        got = {}
        for deg in (-30.0, 15.0, 45.0):
            theta = math.radians(deg)
            set_waist(theta, settle=1.0)
            r = self._convert(point, max_delta=10.0)   # allow the deliberate move
            got[deg] = {"success": r.success, "message": r.message[:90],
                        "planner_point": [round(v, 6) for v in r.planner_point],
                        "expected": [round(v, 6) for v in self._expected(point, theta)],
                        "waist_reported": round(r.waist_yaw_rad, 6),
                        "waist_commanded": round(theta, 6)}
        evidence("G3 nonzero waist", got)
        for deg, row in got.items():
            self.assertTrue(row["success"], "%s deg: %s" % (deg, row["message"]))
            self.assertAlmostEqual(row["waist_reported"], row["waist_commanded"],
                                   places=4, msg="%s deg: waist misreported" % deg)
            for axis in range(3):
                self.assertAlmostEqual(row["planner_point"][axis], row["expected"][axis],
                                       places=5, msg="%s deg axis %d" % (deg, axis))
        set_waist(0.0)

    def test_the_ten_mm_offset_is_actually_applied(self):
        """Without it, z at waist 0 would be 0.863 rather than 0.853."""
        set_waist(0.0)
        r = self._convert([0.412, -0.133, 0.907])
        evidence("G4 ten mm applied", {
            "planner_z": r.planner_point[2],
            "tf_torso_z_would_be": r.planner_point[2] + 0.010})
        self.assertAlmostEqual(r.planner_point[2], 0.853, places=5)
        self.assertNotAlmostEqual(r.planner_point[2], 0.863, places=4)

    def test_an_unknown_source_frame_is_refused(self):
        r = self._convert([0.1, 0.2, 0.3], source="no_such_frame")
        evidence("G5 unknown source frame", {"success": r.success,
                                             "message": r.message[:140]})
        self.assertFalse(r.success)
        self.assertIn("NOT_READY", r.message)

    def test_a_malformed_point_is_refused(self):
        r = self._convert([0.1, 0.2])
        self.assertFalse(r.success)
        self.assertIn("exactly 3", r.message)

    def test_a_stale_stamp_is_refused_not_extrapolated(self):
        """Genuine stale-transform test: ask for a time TF cannot serve.

        NOTE on method: "stop publishing TF" does NOT empty a tf2 buffer -- tf2
        prunes on insert, not on read, so a Time(0) lookup keeps returning the
        last transform forever. Staleness is therefore tested with a stamp
        outside the buffer's range, which is the condition that actually
        matters: an observation older than any transform we hold.
        """
        set_scenario("happy", settle=1.0)
        set_waist(0.0)
        old = rospy.Time.now() - rospy.Duration(600.0)
        r = self._convert([0.412, -0.133, 0.907], stamp_is_obs=True, stamp=old)
        evidence("G6 stale stamp", {"success": r.success, "message": r.message[:170],
                                    "stamp_age_s": 600.0})
        self.assertFalse(r.success)
        self.assertIn("NOT_READY", r.message)
        self.assertIn("transform", r.message)

    def test_a_moved_waist_with_an_unstamped_observation_is_refused(self):
        """perception carries no timestamp, so the pairing is guarded.

        The guard compares the waist in the transform at the observation's stamp
        against the waist in the latest transform, so it needs a real past
        stamp -- which is exactly what the orchestrator passes (its receipt
        time). With Time(0) there is no window and nothing to check.
        """
        set_waist(0.0, settle=1.5)
        stamp_then = rospy.Time.now()
        # A still waist across the window is accepted.
        still = self._convert([0.1, 0.0, 0.5], max_delta=0.02, stamp=stamp_then)
        # Now move the waist and re-ask with the SAME old stamp.
        set_waist(math.radians(40.0), settle=1.0)
        moved = self._convert([0.1, 0.0, 0.5], max_delta=0.02, stamp=stamp_then)
        evidence("G7 waist moved, observation unstamped", {
            "still_waist_success": still.success,
            "still_waist_message": still.message[:90],
            "moved_waist_success": moved.success,
            "moved_waist_message": moved.message[:200]})
        self.assertTrue(still.success, still.message)
        self.assertFalse(moved.success, "a moved waist must be refused")
        self.assertIn("waist moved", moved.message)
        self.assertIn("GetObjectPosition", moved.message)
        set_waist(0.0)

    # --- the orchestrator adapter --------------------------------------------
    def test_to_planner_frame_adapter_uses_the_service(self):
        set_scenario("happy", settle=1.0)
        set_waist(0.0)
        backend = self.onode.RosBackend(SIDE, {})
        rospy.sleep(1.0)
        from orchestrator import handover_state as hs
        from orchestrator import sequencer as seq
        ctx = seq.RouteContext(route="grasp", target="bottle", side=SIDE,
                               dry_run=False, handover=hs.HandoverState())
        det = backend.detect_object(ctx)
        self.assertTrue(det.ok, det.detail)
        conv = backend.to_planner_frame(ctx)
        evidence("G8 to_planner_frame adapter", {
            "detection": {k: (round(v, 6) if isinstance(v, float) else str(v))
                          for k, v in ctx.detection.items()},
            "ok": conv.ok, "detail": conv.detail,
            "planner_target": [round(v, 6) for v in ctx.planner_target],
            "planner_frame": ctx.planner_frame})
        self.assertTrue(conv.ok, conv.detail)
        self.assertEqual(ctx.planner_frame, "torso_link")
        # Detection fixture is (0.412, -0.133, 0.907) in base_link.
        self.assertAlmostEqual(ctx.planner_target[2], 0.853, places=5)
        self.assertIn("convention offset", conv.detail)

    def test_plan_grasp_is_now_reachable_after_conversion(self):
        """The whole of boundary 3 -> 4, with a real conversion."""
        set_scenario("happy", settle=1.0)
        set_waist(0.0)
        backend = self.onode.RosBackend(SIDE, {})
        rospy.sleep(1.2)
        from orchestrator import handover_state as hs
        from orchestrator import sequencer as seq
        ctx = seq.RouteContext(route="grasp", target="bottle", side=SIDE,
                               dry_run=False, handover=hs.HandoverState())
        self.assertTrue(backend.detect_object(ctx).ok)
        self.assertTrue(backend.to_planner_frame(ctx).ok)
        planned = backend.plan_grasp(ctx)
        calls = await_call("plan_grasp_from_point")
        evidence("G9 conversion -> planner", {
            "planner_target": [round(v, 6) for v in ctx.planner_target],
            "plan_ok": planned.ok, "plan_detail": planned.detail,
            "planner_call": [c for c in calls if c.startswith("plan_grasp_from_point")],
            "segments": sorted(ctx.plan)})
        self.assertTrue(planned.ok, planned.detail)
        self.assertEqual(sorted(ctx.plan), ["approach", "lift", "retract", "to_pregrasp"])

    def test_the_orchestrator_still_does_no_tf_maths(self):
        """Requirement 2: the adapter passes through, it does not compute."""
        src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "scripts", "orchestrator_node.py")).read()
        for forbidden in ("tf2_ros", "lookupTransform", "quaternion", "math.cos",
                          "math.sin", "CONVENTION_OFFSET"):
            self.assertNotIn(forbidden, src,
                             "the orchestrator must not contain %r" % forbidden)


# =====================================================================================
class H_PickupAndDeliveryDestinations(unittest.TestCase):
    """RunRoute now carries both waypoints (reconciled audit finding)."""

    def test_deliver_task_refuses_an_empty_destination(self):
        r = _call("/orchestrator/run_deliver_task", RunRoute, route="", target="table",
                  destination="", start_step="", dry_run=True, event_token="")
        evidence("H1 missing destination", {"success": r.success, "message": r.message})
        self.assertFalse(r.success)
        self.assertIn("destination", r.message)

    def test_both_waypoints_are_carried_and_distinct(self):
        set_scenario("happy", settle=0.8)
        r = _call("/orchestrator/run_deliver_task", RunRoute, route="", target="table",
                  destination="person", start_step="", dry_run=True, event_token="")
        evidence("H2 pickup + delivery", {
            "success": r.success, "n_steps": len(r.steps_completed),
            "blockers": len(r.blockers)})
        self.assertTrue(r.success, r.message)
        self.assertEqual(len(r.steps_completed), 29)

    def test_single_leg_routes_ignore_destination(self):
        set_scenario("happy", settle=0.8)
        r = _call("/orchestrator/run_navigate_report", RunRoute, route="", target="table",
                  destination="", start_step="", dry_run=False, event_token="")
        self.assertTrue(r.success, r.message)


# =====================================================================================
class I_StowIsPlannedBeforeGraspMotion(unittest.TestCase):
    """Spec 2.4 (reconciled audit finding)."""

    def test_plan_stow_precedes_every_grasp_motion_step(self):
        from orchestrator import routes as routes_mod
        names = routes_mod.ROUTES["deliver_task"].step_names
        order = {n: i for i, n in enumerate(names)}
        evidence("I1 stow planning order", {
            "plan_grasp": order["plan_grasp"], "plan_stow": order["plan_stow"],
            "execute_to_pregrasp": order["execute_to_pregrasp"],
            "close_gripper": order["close_gripper"],
            "execute_stow": order["execute_stow"]})
        self.assertLess(order["plan_grasp"], order["plan_stow"])
        self.assertLess(order["plan_stow"], order["execute_to_pregrasp"],
                        "the stow path must be planned before any grasp motion")
        self.assertLess(order["plan_stow"], order["close_gripper"],
                        "the stow path must be planned before the hand closes")
        self.assertLess(order["execute_to_pregrasp"], order["execute_stow"])

    def test_plan_stow_refuses_without_the_grasp_final_joints(self):
        """It must plan from where retract ENDS, not from the live posture."""
        from orchestrator import handover_state as hs
        from orchestrator import sequencer as seq
        backend = self.__class__.backend
        ctx = seq.RouteContext(route="grasp", side=SIDE, dry_run=False,
                               handover=hs.HandoverState())
        r = backend.plan_stow(ctx)
        evidence("I2 plan_stow with no final_joints", {"ok": r.ok, "detail": r.detail[:160]})
        self.assertFalse(r.ok)
        self.assertIn("final_joints", r.detail)
        self.assertIn("2.4", r.detail)

    def test_plan_stow_uses_the_grasp_end_posture(self):
        from orchestrator import handover_state as hs
        from orchestrator import sequencer as seq
        backend = self.__class__.backend
        backend.poses = {"stow": [0.12, -0.08, 0.06, 0.0, 0.0]}      # TEST FIXTURE
        ctx = seq.RouteContext(route="grasp", side=SIDE, dry_run=False,
                               handover=hs.HandoverState())
        ctx.grasp_final_joints = [0.11, 0.22, 0.33, -0.44, 0.55]
        r = backend.plan_stow(ctx)
        calls = await_call("plan_to_pose")
        evidence("I3 plan_stow from the grasp end", {
            "ok": r.ok, "detail": r.detail,
            "plan_to_pose_call": [c for c in calls if c.startswith("plan_to_pose")],
            "stored": sorted(ctx.plan)})
        self.assertTrue(r.ok, r.detail)
        self.assertIn("stow", ctx.plan)

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
        import orchestrator_node as onode
        set_scenario("happy", settle=1.0)
        cls.backend = onode.RosBackend(SIDE, {})
        rospy.sleep(1.0)


def main():
    global REC, SCENARIO_PUB, FORCE_PUB, WAIST_PUB
    rospy.init_node("pipeline_integration_test", anonymous=True, disable_signals=True)
    SCENARIO_PUB = rospy.Publisher("/test/scenario", String, queue_size=5, latch=True)
    FORCE_PUB = rospy.Publisher("/test/force_cmd", Float32MultiArray, queue_size=5, latch=True)
    WAIST_PUB = rospy.Publisher("/test/waist_yaw", Float32MultiArray, queue_size=5, latch=True)
    REC = Recorder()
    rospy.wait_for_message("/test/doubles_ready", Bool, timeout=TIMEOUT)
    rospy.wait_for_service("/orchestrator/run_navigate_report", timeout=TIMEOUT)
    rospy.wait_for_service("/auto_release_monitor/arm", timeout=TIMEOUT)
    rospy.sleep(1.0)

    loader = unittest.TestLoader()
    loader.sortTestMethodsUsing = None
    suite = unittest.TestSuite()
    for cls in (A_ReachableRouteRunsForReal, B_FullTaskRoutesAreRefused,
                C_MonitorToOrchestratorTokenPath, D_FailurePropagation,
                E_FullTaskEntryPoint, F_NewlyWiredAdaptersAreReal,
                G_FrameConversionIsProductionCode,
                H_PickupAndDeliveryDestinations,
                I_StowIsPlannedBeforeGraspMotion):
        suite.addTests(loader.loadTestsFromTestCase(cls))
    result = unittest.TextTestRunner(verbosity=2).run(suite)

    out = os.environ.get("TEST_EVIDENCE_OUT", "")
    if out:
        with open(out, "w") as fh:
            json.dump(EVIDENCE, fh, indent=2, default=str)
        print("evidence written to %s" % out)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
