"""ROS-free tests for handover_node's helpers.

    cd handover && PYTHONPATH=src python3 -m unittest discover -s test -p test_handover_flow.py -v
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from handover.handover_flow import interp_joint_waypoints, unflatten  # noqa: E402


class Flow(unittest.TestCase):
    def test_unflatten(self):
        self.assertEqual(unflatten([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], 2), [[1, 2, 3, 4, 5], [6, 7, 8, 9, 10]])
        self.assertEqual(unflatten([1, 2, 3], 1), [])
        self.assertEqual(unflatten([], 0), [])

    def test_interp_caps_step_and_ends_at_target(self):
        a = [0.0] * 5
        b = [math.radians(35.0), 0, 0, math.radians(-20.0), 0]
        wps = interp_joint_waypoints(a, b, 10.0)
        self.assertEqual(len(wps), 4)
        self.assertEqual(wps[-1], b)
        prev = a
        for wp in wps:
            self.assertLessEqual(max(abs(x - y) for x, y in zip(wp, prev)), math.radians(10.0) + 1e-9)
            prev = wp

    def test_interp_degenerate(self):
        self.assertEqual(interp_joint_waypoints([1, 2, 3, 4, 5], [1, 2, 3, 4, 5], 10.0), [[1, 2, 3, 4, 5]])
        self.assertEqual(interp_joint_waypoints([0] * 5, [1] * 5, 0.0), [[1] * 5])


if __name__ == "__main__":
    unittest.main()
