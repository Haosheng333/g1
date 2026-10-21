"""After a successful grasp the arm lifts, retracts to x = retract_x_target and
returns to the home joint config, driven from the planner's keyframes.

    cd arm_grasp && PYTHONPATH=src:$PYTHONPATH python3 -m unittest discover -s test -p test_post_grasp_leg.py -v
"""
import logging
import os
import sys
import types
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))


class _Resp:
    def __init__(self, ok, q, msg=""):
        self.success, self.current_joint_angles, self.message = ok, list(q), msg


class PostGraspLeg(unittest.TestCase):
    def setUp(self):
        try:
            from arm_grasp.grasp_pipeline import run_post_grasp_leg  # needs rospy importable
        except ImportError as exc:
            raise unittest.SkipTest(str(exc))
        self.run = run_post_grasp_leg
        self.calls = []

        def lift_move(arm, joints, wait, timeout):
            self.calls.append((arm, list(joints)))
            return _Resp(True, joints)

        self.ctx = types.SimpleNamespace(
            retract_after_grasp=True, post_grasp_home=True, lift_move=lift_move,
            move_arm_service_name="/g1_arm_simple/move_arm_joints", logger=logging.getLogger("t"))
        self.plan = {"retract_joints": np.array([0.1, 0.2, 0.3, 0.4, 0.5]),
                     "home_joints": np.array([0.35, 0.23, 0.0, 0.87, 0.0])}

    def test_retract_then_home_in_order(self):
        result = {"message": "", "final_joint_angles": []}
        ok, done = self.run(self.ctx, "left", self.plan, True, 6.0, result, "t")
        self.assertTrue(ok)
        self.assertEqual(done, ["lift", "retract", "home"])
        self.assertEqual([c[1] for c in self.calls], [self.plan["retract_joints"].tolist(), self.plan["home_joints"].tolist()])
        self.assertEqual(result["final_joint_angles"], self.plan["home_joints"].tolist())

    def test_retract_skipped_when_not_planned(self):
        self.plan["retract_joints"] = None
        ok, done = self.run(self.ctx, "left", self.plan, True, 6.0, {"message": ""}, "t")
        self.assertTrue(ok)
        self.assertEqual(done, ["lift", "home"])

    def test_home_can_be_disabled(self):
        self.ctx.post_grasp_home = False
        ok, done = self.run(self.ctx, "right", self.plan, True, 6.0, {"message": ""}, "t")
        self.assertTrue(ok)
        self.assertEqual(done, ["lift", "retract"])

    def test_failure_stops_and_reports(self):
        def failing(arm, joints, wait, timeout):
            self.calls.append((arm, list(joints)))
            return _Resp(False, joints, "timeout")
        self.ctx.lift_move = failing
        result = {"message": ""}
        ok, done = self.run(self.ctx, "left", self.plan, True, 6.0, result, "t")
        self.assertFalse(ok)
        self.assertEqual(done, ["lift"])
        self.assertIn("failed at retract motion: timeout", result["message"])
        self.assertEqual(len(self.calls), 1, "must not continue to home after a failed retract")


if __name__ == "__main__":
    unittest.main()
