"""The voxel obstacle map must use the same camera extrinsics as perception.

    cd arm_grasp && PYTHONPATH=src:$PYTHONPATH python3 -m unittest discover -s test -p test_voxel_calibration.py -v
(keep $PYTHONPATH: the adapter imports rospy from the sourced ROS overlay)
"""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
PERCEPTION_CONFIG = os.path.join(HERE, "..", "..", "perception", "config")
FITTED = os.path.join(PERCEPTION_CONFIG, "d455_extrinsics_20261017_fitted_20261021.yaml")


class VoxelCalibration(unittest.TestCase):
    def setUp(self):
        try:
            import yaml  # noqa: F401
            from arm_grasp.voxel_planner_adapter import optical_to_root_from_calibration  # needs rospy importable
        except ImportError as exc:
            raise unittest.SkipTest(str(exc))
        self.load = optical_to_root_from_calibration

    def test_matches_perception_loader(self):
        R, t = self.load(FITTED)
        sys.path.insert(0, os.path.join(HERE, "..", "..", "perception", "src"))
        try:
            from perception.m1_perception import extrinsics_from_calibration_yaml
        except ImportError as exc:
            raise unittest.SkipTest(str(exc))
        ext = extrinsics_from_calibration_yaml(FITTED)
        np.testing.assert_allclose(R, ext.R_base_opt)
        np.testing.assert_allclose(t, ext.t_base_opt)

    def test_rejects_other_parent_frame(self):
        with self.assertRaises(ValueError):
            self.load(FITTED, expected_parent="base_link")

    def test_same_pixel_lands_on_the_same_point(self):
        """A depth pixel back-projected by the adapter's maths and by perception agree."""
        R, t = self.load(FITTED)
        fx, fy, cx, cy = 645.0, 644.0, 640.0, 360.0
        u, v, z = 700.0, 500.0, 0.45
        p_opt = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
        p_adapter = p_opt @ R.T + t  # as in _depth_to_obstacle_points
        p_perception = R @ p_opt + t  # as in optical_to_base
        np.testing.assert_allclose(p_adapter, p_perception)
        self.assertLess(p_adapter[2], 0.0, "a desk pixel must land below the torso origin")


if __name__ == "__main__":
    unittest.main()
