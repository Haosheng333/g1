"""Offline tests for the arm_planning service conversion layer.

No ROS, no torch, no numpy: these cover exactly the glue this package adds
(validation, flattening, joint ordering, failure handling), not the planner
algorithms, which have their own suite under arm_planning/tests/.

    cd arm_planning && python3 -m unittest discover -s test -v
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from arm_planning import plan_conversion as pc  # noqa: E402

REPO_ARM_PLANNING = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


class JointOrdering(unittest.TestCase):
    """Doc section 2.3: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll."""

    def test_joint_order_is_the_specification_order(self):
        self.assertEqual(
            pc.JOINT_ORDER,
            ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"),
        )
        self.assertEqual(pc.DOF, 5)

    def test_expected_joint_names_match_arm_kinematics_yaml(self):
        """Our order must equal the planner's own config, or paths get transposed."""
        import yaml

        cfg_path = os.path.join(REPO_ARM_PLANNING, "config", "arm_kinematics.yaml")
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        for arm in pc.ARMS:
            self.assertEqual(
                list(cfg[arm]["joint_names"]),
                pc.expected_joint_names(arm),
                "joint order drift between plan_conversion and arm_kinematics.yaml",
            )

    def test_joint_order_matches_arm_control_execution_order(self):
        """arm_control writes JointIndex.*Pitch/Roll/Yaw/Elbow/WristRoll in this order.

        A mismatch here would send a correct plan to the wrong joints, so it is
        pinned rather than trusted to comments.
        """
        controller = os.path.join(
            REPO_ARM_PLANNING, "..", "arm_control", "src", "arm_control", "arm_controller.py"
        )
        with open(controller) as f:
            source = f.read()
        start = source.index("left_joints = [")
        block = source[start:source.index("]", start)]
        suffixes = ["ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "Elbow", "WristRoll"]
        found = [s for s in suffixes if ("Left" + s) in block]
        self.assertEqual(found, suffixes, "arm_control joint order differs from JOINT_ORDER")
        # Same five, same sequence, as the camel-case form of JOINT_ORDER.
        camel = ["".join(p.capitalize() for p in j.split("_")) for j in pc.JOINT_ORDER]
        self.assertEqual(camel, suffixes)

    def test_expected_joint_names_rejects_unknown_arm(self):
        with self.assertRaises(ValueError):
            pc.expected_joint_names("middle")


class RequestValidation(unittest.TestCase):
    """Doc 12.2 rejection conditions checkable before the planner runs."""

    def test_accepts_a_well_formed_pose_request(self):
        self.assertIsNone(
            pc.validate_plan_to_pose_request("left", [0, 0, 0, 0, 0], [0.3, 0.15, 0.0, 0.0, 0.0])
        )

    def test_accepts_a_well_formed_grasp_request(self):
        self.assertIsNone(
            pc.validate_plan_grasp_request("right", [0, 0, 0, 0, 0], [0.3, -0.15, 0.0])
        )

    def test_rejects_unknown_arm(self):
        msg = pc.validate_plan_to_pose_request("middle", [0] * 5, [0] * 5)
        self.assertIn("left", msg)
        self.assertIn("right", msg)

    def test_rejects_wrong_joint_count(self):
        for bad in ([0, 0, 0, 0], [0] * 6, []):
            msg = pc.validate_plan_to_pose_request("left", bad, [0] * 5)
            self.assertIn("current_joints", msg)
            self.assertIn("exactly 5", msg)

    def test_rejects_wrong_pose_length(self):
        msg = pc.validate_plan_to_pose_request("left", [0] * 5, [0.3, 0.1, 0.0])
        self.assertIn("target_pose", msg)

    def test_rejects_wrong_object_length(self):
        msg = pc.validate_plan_grasp_request("left", [0] * 5, [0.3, 0.1])
        self.assertIn("object_xyz", msg)

    def test_rejects_non_finite_values(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            msg = pc.validate_plan_to_pose_request("left", [0, 0, bad, 0, 0], [0] * 5)
            self.assertIn("finite", msg)

    def test_rejects_non_numeric_values(self):
        msg = pc.validate_plan_to_pose_request("left", [0, 0, "x", 0, 0], [0] * 5)
        self.assertIn("must be a number", msg)

    def test_rejects_booleans_disguised_as_numbers(self):
        msg = pc.validate_plan_to_pose_request("left", [0, 0, True, 0, 0], [0] * 5)
        self.assertIn("must be a number", msg)


class PathFlattening(unittest.TestCase):
    """Doc 11.4: response carries a flat row-major path plus its waypoint count."""

    def test_flattens_row_major_with_count(self):
        path = [[1, 2, 3, 4, 5], [6, 7, 8, 9, 10]]
        flat, count = pc.flatten_path(path)
        self.assertEqual(count, 2)
        self.assertEqual(flat, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

    def test_empty_path_flattens_to_nothing(self):
        self.assertEqual(pc.flatten_path([]), ([], 0))
        self.assertEqual(pc.flatten_path(None), ([], 0))

    def test_round_trip_through_unflatten(self):
        path = [[0.1, 0.2, 0.3, 0.4, 0.5], [1.1, 1.2, 1.3, 1.4, 1.5], [2.0] * 5]
        flat, count = pc.flatten_path(path)
        self.assertEqual(pc.unflatten_path(flat, count), path)

    def test_rejects_ragged_path(self):
        with self.assertRaises(ValueError) as ctx:
            pc.flatten_path([[0] * 5, [0] * 4])
        self.assertIn("waypoint 1", str(ctx.exception))

    def test_rejects_non_finite_waypoint(self):
        with self.assertRaises(ValueError) as ctx:
            pc.flatten_path([[0, 0, float("nan"), 0, 0]])
        self.assertIn("not finite", str(ctx.exception))

    def test_unflatten_rejects_inconsistent_count(self):
        with self.assertRaises(ValueError):
            pc.unflatten_path([0] * 10, 3)


class SegmentFlattening(unittest.TestCase):
    """Doc 6.1 / 11.4: four segments, fixed order, concatenated blocks."""

    def _segments(self):
        return {
            "to_pregrasp": {"joint_path": [[0] * 5, [1] * 5]},
            "approach": {"joint_path": [[2] * 5]},
            "lift": {"joint_path": [[3] * 5, [4] * 5, [5] * 5]},
            "retract": {"joint_path": [[6] * 5]},
        }

    def test_segment_order_is_the_specification_order(self):
        self.assertEqual(pc.SEGMENT_ORDER, ("to_pregrasp", "approach", "lift", "retract"))

    def test_flattens_in_specification_order_regardless_of_dict_order(self):
        segments = self._segments()
        shuffled = {k: segments[k] for k in ("retract", "lift", "to_pregrasp", "approach")}
        names, counts, flat = pc.flatten_segments(shuffled)
        self.assertEqual(names, list(pc.SEGMENT_ORDER))
        self.assertEqual(counts, [2, 1, 3, 1])
        self.assertEqual(len(flat), sum(counts) * pc.DOF)

    def test_segments_can_be_split_back_out(self):
        names, counts, flat = pc.flatten_segments(self._segments())
        offset = 0
        rebuilt = {}
        for name, count in zip(names, counts):
            length = count * pc.DOF
            rebuilt[name] = pc.unflatten_path(flat[offset:offset + length], count)
            offset += length
        self.assertEqual(offset, len(flat))
        self.assertEqual(rebuilt["approach"], [[2] * 5])
        self.assertEqual(rebuilt["lift"], [[3] * 5, [4] * 5, [5] * 5])

    def test_rejects_missing_segment(self):
        segments = self._segments()
        del segments["lift"]
        with self.assertRaises(ValueError) as ctx:
            pc.flatten_segments(segments)
        self.assertIn("lift", str(ctx.exception))

    def test_rejects_empty_segment(self):
        segments = self._segments()
        segments["approach"] = {"joint_path": []}
        with self.assertRaises(ValueError) as ctx:
            pc.flatten_segments(segments)
        self.assertIn("no waypoints", str(ctx.exception))

    def test_accepts_objects_with_a_joint_path_attribute(self):
        class Segment(object):
            def __init__(self, path):
                self.joint_path = path

        segments = {k: Segment(v["joint_path"]) for k, v in self._segments().items()}
        names, counts, _ = pc.flatten_segments(segments)
        self.assertEqual(names, list(pc.SEGMENT_ORDER))
        self.assertEqual(counts, [2, 1, 3, 1])


class OptionalFloatConversion(unittest.TestCase):
    """A ROS float64 cannot be None, so unmeasured verification becomes NaN."""

    def test_none_becomes_nan(self):
        self.assertTrue(math.isnan(pc.nan_if_none(None)))

    def test_numbers_pass_through(self):
        self.assertEqual(pc.nan_if_none(0.00464), 0.00464)
        self.assertEqual(pc.nan_if_none(0), 0.0)


if __name__ == "__main__":
    unittest.main()
