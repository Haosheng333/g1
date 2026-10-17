#!/usr/bin/env python3
"""approach_forward: front-gap measurement and the stop rules (no ROS)."""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from approach_forward import ABORT, STOPPED_GAP, SUCCESS, front_gap, step  # noqa: E402,F401

CFG = {"speed": 0.12, "min_front_gap": 0.03, "stop_early": 0.02, "yaw_gain": 1.5, "max_wz": 0.3}


class FrontGap(unittest.TestCase):
    def test_table_edge_ahead(self):
        pts = np.array([[0.40, 0.05, 0.0], [0.50, -0.10, 0.0]])
        gap, n = front_gap(pts, np.array([0.75, 0.75]), 0.0)
        self.assertAlmostEqual(gap, 0.30)
        self.assertEqual(n, 2)

    def test_points_outside_the_corridor_or_band_are_ignored(self):
        pts = np.array([[0.30, 0.50, 0.0],     # beside the robot
                        [0.30, 0.00, 0.0],     # floor
                        [0.30, 0.00, 0.0],     # ceiling
                        [0.05, 0.00, 0.0]])    # inside the body
        gap, n = front_gap(pts, np.array([0.75, 0.05, 2.0, 0.75]), 0.0)
        self.assertEqual(gap, float("inf"))
        self.assertEqual(n, 0)


class Step(unittest.TestCase):
    def test_walks_forward_until_the_distance(self):
        res, vx, wz, _ = step((0, 0, 0), (0.05, 0, 0), 0.20, 1.0, CFG)
        self.assertIsNone(res)
        self.assertAlmostEqual(vx, 0.12)
        res, vx, _, d = step((0, 0, 0), (0.185, 0, 0), 0.20, 1.0, CFG)
        self.assertEqual(res, SUCCESS)
        self.assertEqual(vx, 0.0)

    def test_gap_guard_stops_forward_but_not_backing_out(self):
        self.assertEqual(step((0, 0, 0), (0.1, 0, 0), 0.20, 0.02, CFG)[0], STOPPED_GAP)
        res, vx, _, _ = step((0, 0, 0), (0, 0, 0), -0.20, 0.0, CFG)
        self.assertIsNone(res)
        self.assertAlmostEqual(vx, -0.12)

    def test_distance_is_measured_along_the_start_heading(self):
        yaw = math.radians(-144.5)
        p = (0.15 * math.cos(yaw), 0.15 * math.sin(yaw), yaw)
        self.assertAlmostEqual(step((0, 0, yaw), p, 0.20, 1.0, CFG)[3], 0.15)

    def test_heading_is_held(self):
        _, _, wz, _ = step((0, 0, 0), (0.05, 0, 0.1), 0.20, 1.0, CFG)
        self.assertLess(wz, 0.0)


if __name__ == "__main__":
    unittest.main()
