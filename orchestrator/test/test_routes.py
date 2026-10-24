"""Route declarations: order, resume, and the blocked grasp route."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from orchestrator import capabilities as caps  # noqa: E402
from orchestrator import routes as r  # noqa: E402


class RouteCatalogue(unittest.TestCase):
    def test_the_five_routes_exist(self):
        """auto_release was deleted: its steps live inside deliver now.
        fetch_deliver is the spoken task on the modules that have run on the robot."""
        self.assertEqual(sorted(r.ROUTES),
                         ["deliver", "deliver_task", "fetch_deliver", "grasp",
                          "navigate_report"])
        self.assertFalse(hasattr(r, "AUTO_RELEASE"))

    def test_unknown_route_raises_listing_the_known_ones(self):
        with self.assertRaises(KeyError) as ctx:
            r.resolve_route("fly")
        self.assertIn("grasp", str(ctx.exception))

    def test_every_step_names_a_capability_that_exists(self):
        for route in r.ROUTES.values():
            for step in route.steps:
                if step.capability:
                    caps.capability(step.capability)   # raises if unknown

    def test_every_required_capability_exists(self):
        for route in r.ROUTES.values():
            for key in route.requires:
                caps.capability(key)

    def test_step_names_are_unique_within_a_route(self):
        for route in r.ROUTES.values():
            names = route.step_names
            self.assertEqual(len(names), len(set(names)), route.name)


class GraspIsBlocked(unittest.TestCase):
    """The full grasp needs a frame contract nobody owns yet."""

    def test_grasp_requires_the_conversion_and_the_camera_extrinsics(self):
        """frame_contract was split: one half is implemented, one is not."""
        self.assertIn(caps.PLANNER_FRAME_CONVERSION, r.GRASP.requires)
        self.assertIn(caps.PERCEPTION_EXTRINSICS, r.GRASP.requires)

    def test_grasp_is_not_ready_in_principle(self):
        self.assertFalse(r.GRASP.ready_in_principle)
        blockers = r.GRASP.blockers()
        # frame contract + the stow pose a grasp must leave the arm in.
        self.assertEqual(len(blockers), 2, blockers)
        joined = " | ".join(blockers)
        self.assertIn("torso_link", joined)
        self.assertIn("base_link", joined)
        self.assertIn("stow", joined)

    def test_delivery_declares_its_known_missing_requirements(self):
        """So preflight refuses up front instead of after the hand opens."""
        for key in (caps.HANDOVER_POSE, caps.RELEASE_COMPLETION, caps.HOME_POSE):
            self.assertIn(key, r.DELIVER.requires)
        self.assertFalse(r.DELIVER.ready_in_principle)
        joined = " | ".join(r.DELIVER.blockers())
        self.assertIn("no handover pose", joined)
        self.assertIn("no home pose", joined)
        self.assertIn("require_settle", joined)

    def test_the_10mm_disagreement_is_now_resolved_in_code_not_a_blocker(self):
        """It moved from a blocker to a documented, tested convention offset."""
        import sys as _sys
        from pathlib import Path as _Path
        pkg = _Path(__file__).resolve().parents[2] / "arm_planning" / "src"
        _sys.path.insert(0, str(pkg))
        from arm_planning import frame_conversion as fc
        self.assertAlmostEqual(fc.TF_WAIST_Z, 0.044, places=9)
        self.assertAlmostEqual(fc.PLANNER_WAIST_Z, 0.054, places=9)
        self.assertAlmostEqual(fc.CONVENTION_OFFSET[2], 0.010, places=9)
        self.assertIn("0.044", fc.__doc__)
        self.assertIn("0.054", fc.__doc__)
        # And it is no longer reported as an unresolved grasp blocker.
        self.assertNotIn("10 mm", " ".join(r.GRASP.blockers()))

    def test_deliver_needs_the_planner_but_no_frame_conversion(self):
        """It plans to a CONFIGURED pose already in the planner's frame.

        It converts no observation and runs no perception, so neither the
        conversion nor the camera extrinsics apply. Both reach deliver_task
        through the grasp phase instead.
        """
        self.assertIn(caps.ARM_PLANNING, r.DELIVER.requires)
        self.assertNotIn(caps.PLANNER_FRAME_CONVERSION, r.DELIVER.requires)
        self.assertNotIn(caps.PERCEPTION_EXTRINSICS, r.DELIVER.requires)
        self.assertFalse(r.DELIVER.ready_in_principle)
        self.assertIn(caps.PLANNER_FRAME_CONVERSION, r.ROUTES["deliver_task"].requires)
        self.assertIn(caps.PERCEPTION_EXTRINSICS, r.ROUTES["deliver_task"].requires)

    def test_the_other_routes_are_ready_in_principle(self):
        for route in (r.NAVIGATE_REPORT,):
            self.assertTrue(route.ready_in_principle, route.name)
            self.assertEqual(route.blockers(), [])

    def test_navigate_report_does_not_need_any_frame_work(self):
        """It reports the detection; it never converts it into a planner target."""
        self.assertNotIn(caps.PLANNER_FRAME_CONVERSION, r.NAVIGATE_REPORT.requires)
        self.assertNotIn(caps.PERCEPTION_EXTRINSICS, r.NAVIGATE_REPORT.requires)
        self.assertNotIn(caps.ARM_PLANNING, r.NAVIGATE_REPORT.requires)
        self.assertNotIn(caps.ARM_CONTROL, r.NAVIGATE_REPORT.requires)


class GraspOrder(unittest.TestCase):
    """Doc 13.3: to_pregrasp -> open -> approach -> close -> lift -> retract."""

    def test_segment_and_gripper_order(self):
        names = r.GRASP.step_names
        expected = [
            "execute_to_pregrasp", "open_gripper", "execute_approach",
            "close_gripper", "execute_lift", "execute_retract",
        ]
        self.assertEqual([n for n in names if n in expected], expected)

    def test_the_whole_grasp_is_planned_before_any_arm_or_hand_motion(self):
        names = r.GRASP.step_names
        plan_index = names.index("plan_grasp")
        for step in r.GRASP.steps:
            if step.capability in (caps.ARM_CONTROL, caps.HAND):
                self.assertGreater(
                    names.index(step.name), plan_index,
                    "%s moves the arm/hand before plan_grasp" % step.name,
                )

    def test_detection_precedes_planning(self):
        names = r.GRASP.step_names
        self.assertLess(names.index("detect_object"), names.index("plan_grasp"))

    def test_motion_gate_is_opened_before_any_motion_step(self):
        for route in r.ROUTES.values():
            names = route.step_names
            if "enable_motion" not in names:
                continue
            gate = names.index("enable_motion")
            for step in route.steps:
                if step.motion and step.name != "enable_motion":
                    self.assertGreater(names.index(step.name), gate, "%s/%s" % (route.name, step.name))


class StartStepResume(unittest.TestCase):
    def test_empty_start_step_runs_everything(self):
        skipped, to_run = r.plan_steps(r.DELIVER, "")
        self.assertEqual(skipped, [])
        self.assertEqual([s.name for s in to_run], r.DELIVER.step_names)

    def test_resume_skips_the_earlier_steps(self):
        skipped, to_run = r.plan_steps(r.NAVIGATE_REPORT, "detect_object")
        self.assertEqual(
            [s.name for s in skipped],
            ["await_localization", "enable_motion", "navigate"],
        )
        self.assertEqual([s.name for s in to_run], ["detect_object", "report"])

    def test_resume_at_the_first_step_skips_nothing(self):
        skipped, to_run = r.plan_steps(r.NAVIGATE_REPORT, "await_localization")
        self.assertEqual(skipped, [])
        self.assertEqual(len(to_run), len(r.NAVIGATE_REPORT.steps))

    def test_resume_at_the_last_step_runs_only_it(self):
        skipped, to_run = r.plan_steps(r.NAVIGATE_REPORT, "report")
        self.assertEqual(len(to_run), 1)
        self.assertEqual(to_run[0].name, "report")

    def test_unknown_start_step_raises_instead_of_running_everything(self):
        """A typo must not silently repeat motion that already happened."""
        with self.assertRaises(KeyError) as ctx:
            r.plan_steps(r.NAVIGATE_REPORT, "detect_objct")
        self.assertIn("detect_objct", str(ctx.exception))
        self.assertIn("await_localization", str(ctx.exception))

    def test_skipped_plus_to_run_is_always_the_whole_route(self):
        for route in r.ROUTES.values():
            for name in route.step_names:
                skipped, to_run = r.plan_steps(route, name)
                self.assertEqual(
                    [s.name for s in skipped] + [s.name for s in to_run], route.step_names
                )


class FetchDeliverIsTheSpokenTask(unittest.TestCase):
    """navigate -> grasp -> navigate -> handover on arm_grasp + handover_node."""

    STEPS = r.FETCH_DELIVER.step_names

    def test_it_is_ready_in_principle(self):
        """Nothing it requires is an unresolved decision: all runtime probes."""
        self.assertTrue(r.FETCH_DELIVER.ready_in_principle, r.FETCH_DELIVER.blockers())
        for key in r.FETCH_DELIVER.requires:
            self.assertEqual(caps.capability(key).unresolved, "", key)

    def test_it_uses_the_tested_stack_not_arm_planning(self):
        self.assertIn(caps.ARM_GRASP, r.FETCH_DELIVER.requires)
        self.assertIn(caps.REMOTE_HANDS, r.FETCH_DELIVER.requires)
        self.assertIn(caps.HANDOVER_NODE, r.FETCH_DELIVER.requires)
        for key in (caps.ARM_PLANNING, caps.ARM_CONTROL, caps.HAND, caps.TACTILE_HANDOVER,
                    caps.PLANNER_FRAME_CONVERSION, caps.PERCEPTION_EXTRINSICS,
                    caps.STOW_POSE, caps.HANDOVER_POSE, caps.HOME_POSE):
            self.assertNotIn(key, r.FETCH_DELIVER.requires, key)

    def test_the_order_is_main_process_s_plan(self):
        for before, after in (
            ("enable_motion", "navigate_to_pickup"),
            ("navigate_to_pickup", "settle_at_pickup"),
            ("settle_at_pickup", "detect_object"),         # camera read on a still base
            ("detect_object", "plan_grasp_point"),
            ("plan_grasp_point", "execute_grasp"),          # planned before anything moves
            ("execute_grasp", "record_held_object"),        # state written after the grasp
            ("record_held_object", "hand_back_arms"),
            ("hand_back_arms", "navigate_to_recipient"),    # @walking_task: hand back first
            ("navigate_to_recipient", "settle_at_recipient"),
            ("settle_at_recipient", "handover_start"),      # arm plans from a still base
            ("handover_start", "await_handover"),
            ("await_handover", "hand_back_arms_after_handover"),
        ):
            self.assertLess(self.STEPS.index(before), self.STEPS.index(after),
                            "%s must precede %s" % (before, after))

    def test_no_arm_moves_before_the_grasp_is_planned(self):
        planned = self.STEPS.index("plan_grasp_point")
        for step in r.FETCH_DELIVER.steps[:planned]:
            self.assertNotEqual(step.capability, caps.ARM_GRASP, step.name)

    def test_both_waypoints_are_required_and_resolved_up_front(self):
        self.assertIn("fetch_deliver", r.ROUTES_NEEDING_DESTINATION)
        self.assertIn("fetch_deliver", r.ROUTES_USING_STANDING_WAYPOINTS)

    def test_the_pickup_and_delivery_drives_are_distinct_actions(self):
        actions = {s.name: s.action for s in r.FETCH_DELIVER.steps}
        self.assertEqual(actions["navigate_to_pickup"], "navigate_standing")
        self.assertEqual(actions["navigate_to_recipient"], "navigate_standing_destination")
        # Never the semantic.yaml interface: a standing pose takes no approach offset.
        self.assertNotIn("navigate", actions.values())


if __name__ == "__main__":
    unittest.main()
