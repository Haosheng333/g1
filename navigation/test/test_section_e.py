"""Section E conformance (items 17-21), run against the CURRENT navigation stack (no ROS).

    cd navigation/test && python3 -m unittest test_section_e -v

17  holonomic 3-phase DWA              NAVIGATION -> APPROACH -> TERMINAL, vx/vy/wz, per-phase limits
18  footprint collision checks        5 circles r=0.10 m over the 0.60 m body, heading dependent, in the cost
19  arrival 0.10 m AND 0.1 rad        node reports SUCCESS only inside the spec
20  tolerances come from navigation.yaml and are tighter than the spec
21  limits stay inside what the walking controller accepts (cmd_vel_gate) and obey phase order
"""
import math
import os
import re
import sys
import unittest

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "slam", "src"))

import test_navigation_node as tn  # noqa: E402  (installs the rospy stubs)
from costmap import Costmap, OBSTACLE  # noqa: E402
from g1_slam import grid_utils as gu  # noqa: E402
from local_planner import (APPROACH, NAVIGATION, SPEC_XY_TOL, SPEC_YAW_TOL, TERMINAL,  # noqa: E402
                           DwaController, wrap)

RES = 0.05
with open(os.path.join(HERE, "..", "config", "navigation.yaml")) as _f:
    CFG = yaml.safe_load(_f)
DWA = CFG["dwa"]


def open_room():
    g = np.zeros((120, 160), np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OBSTACLE
    return Costmap.from_grid(gu.compose_tiers(g, RES), RES)


class Item17Holonomic3Phase(unittest.TestCase):
    def setUp(self):
        self.cm = open_room()

    def test_phase_follows_distance_to_goal(self):
        d = DwaController()
        path = [(1.0, 3.0), (6.0, 3.0)]
        seen = []
        for x in (1.0, 5.0, 5.4, 5.6, 5.85):                 # goal at x = 6.0: 5.0 m, 1.0, 0.6, 0.4, 0.15 m away
            seen.append(d.compute((x, 3.0, 0.0), (0, 0, 0), path, (6.0, 3.0), 0.0, self.cm).phase)
        self.assertEqual(seen, [NAVIGATION, NAVIGATION, APPROACH, APPROACH, TERMINAL])

    def test_phase_limits_are_respected(self):
        d = DwaController()
        lim = {NAVIGATION: DWA["nav_limits"], APPROACH: DWA["approach_limits"], TERMINAL: DWA["terminal_limits"]}
        for x in (1.0, 5.4, 5.85):
            r = d.compute((x, 3.0, 0.0), (0, 0, 0), [(1.0, 3.0), (6.0, 3.0)], (6.0, 3.0), 0.0, self.cm)
            vx, vy, wz = lim[r.phase]
            self.assertLessEqual(abs(r.vy), vy + 1e-9)
            self.assertLessEqual(r.vx, vx + 1e-9)
            self.assertLessEqual(abs(r.wz), wz + 1e-9)

    def test_holonomic_sideways_goal_uses_vy_not_a_turn(self):
        d = DwaController()
        r = d.compute((3.0, 3.0, 0.0), (0, 0, 0), [(3.0, 3.0), (3.0, 3.6)], (3.0, 3.6), 0.0, self.cm)
        self.assertEqual(r.phase, APPROACH)
        self.assertGreater(r.vy, 0.02)                          # strafes left
        self.assertLess(abs(r.vx), 0.05)

    def test_terminal_translates_then_rotates(self):
        d = DwaController()
        r = d.compute((3.0, 3.0, 0.0), (0, 0, 0), [(3.0, 3.0), (3.15, 3.0)], (3.15, 3.0), 1.0, self.cm)
        self.assertEqual(r.phase, TERMINAL)
        self.assertGreater(r.vx, 0.0)                           # closes the 0.15 m first
        r = d.compute((3.15, 3.0, 0.0), (0, 0, 0), [(3.0, 3.0), (3.15, 3.0)], (3.15, 3.0), 1.0, self.cm)
        self.assertEqual((r.vx, r.vy), (0.0, 0.0))              # position done: rotate only
        self.assertGreater(r.wz, 0.0)

    def test_acceleration_limits(self):
        d = DwaController()
        r = d.compute((1.0, 3.0, 0.0), (0, 0, 0), [(1.0, 3.0), (6.0, 3.0)], (6.0, 3.0), 0.0, self.cm)
        dt = d.cfg.get("dt", 0.1)
        ax, ay, aw = DWA["acc"]
        self.assertLessEqual(r.vx, ax * dt * 2 + 1e-9)          # first command cannot jump to full speed
        self.assertLessEqual(abs(r.wz), aw * dt * 2 + 1e-9)


class Item18Footprint(unittest.TestCase):
    def test_five_circles_cover_the_body(self):
        cm = open_room()
        self.assertEqual(len(cm.circle_y), 5)
        self.assertAlmostEqual(float(max(cm.circle_y) - min(cm.circle_y)) + 2 * cm.cfg["circle_radius"], 0.50)

    def test_gap_fits_only_with_the_right_heading(self):
        g = np.zeros((120, 160), np.int8)
        g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OBSTACLE
        g[:50, 80] = OBSTACLE                                   # wall x = 4.0 with a 0.5 m gap, y 2.5 .. 3.0
        g[60:, 80] = OBSTACLE
        cm = Costmap.from_grid(gu.compose_tiers(g, RES), RES)
        gate_x, gate_y = 4.025, 2.75
        self.assertTrue(cm.footprint_collision(gate_x, gate_y, 0.0))            # 0.50 m wide body, 0.50 gap
        self.assertFalse(cm.footprint_collision(gate_x, gate_y, math.pi / 2))   # turned sideways it fits

    def test_collision_cost_is_in_the_dwa_not_only_the_planner(self):
        """DWA never drives the footprint into a wall that is 0.4 m ahead."""
        cm = open_room()
        d = DwaController()
        r = d.compute((0.65, 3.0, math.pi), (0, 0, 0), [(0.65, 3.0), (0.2, 3.0)], (0.2, 3.0), math.pi, cm)
        if not r.blocked:
            x = 0.65 + r.vx * math.cos(math.pi) * 0.5
            self.assertFalse(cm.footprint_collision(x, 3.0, math.pi))


class Item19Arrival(unittest.TestCase):
    def setUp(self):
        self.case = tn.NavTest("test_rviz_goal_success_with_yaw")
        self.case.setUp()

    def tearDown(self):
        self.case.tearDown()

    def test_spec_constants(self):
        self.assertEqual((SPEC_XY_TOL, SPEC_YAW_TOL), (0.10, 0.1))

    def test_success_needs_position_and_yaw(self):
        c = self.case
        n = c.node()
        n.state_cb(tn.nn.Bool(data=True))
        n.goal_cb(c.goal_msg(2.0, 1.0, 1.0))
        c.step(n, 60)
        self.assertEqual(c.statuses()[-1], "SUCCESS")
        x, y, yaw = tn.WORLD.pose
        self.assertLess(math.hypot(x - 2.0, y - 1.0), SPEC_XY_TOL)
        self.assertLess(abs(wrap(yaw - 1.0)), SPEC_YAW_TOL)

    def test_no_success_while_yaw_is_off(self):
        c = self.case
        tn.WORLD.pose = (2.0, 1.0, 0.0)                         # already at the goal position, yaw 0.6 rad off
        n = c.node()
        n.state_cb(tn.nn.Bool(data=True))
        n.goal_cb(c.goal_msg(2.0, 1.0, 0.6))
        c.step(n, 1.0)
        self.assertNotIn("SUCCESS", c.statuses())
        c.step(n, 30)
        self.assertEqual(c.statuses()[-1], "SUCCESS")
        self.assertLess(abs(wrap(tn.WORLD.pose[2] - 0.6)), SPEC_YAW_TOL)


class Item20Tolerances(unittest.TestCase):
    def test_internal_tolerances_are_tighter_than_the_spec(self):
        self.assertLess(DWA["xy_tol"], SPEC_XY_TOL)
        self.assertLess(DWA["yaw_tol"], SPEC_YAW_TOL)
        self.assertGreater(CFG["spec_settle_time"], 0.0)

    def test_yaml_values_reach_the_controller(self):
        tn.WORLD.params = {"~semantic_file": "", "~dwa": dict(DWA), "~use_localization_gate": False}
        n = tn.nn.NavigationNode()
        for key in ("xy_tol", "yaw_tol", "terminal_dist", "w_path"):
            self.assertEqual(n.dwa.cfg[key], DWA[key], key)


class Item21Limits(unittest.TestCase):
    def test_phase_order(self):
        for i in range(3):
            self.assertGreaterEqual(DWA["nav_limits"][i], DWA["approach_limits"][i])
            self.assertGreaterEqual(DWA["approach_limits"][i], DWA["terminal_limits"][i])

    def test_inside_cmd_vel_gate(self):
        with open(os.path.join(HERE, "..", "scripts", "cmd_vel_gate.py")) as f:
            src = f.read()
        gate = [float(re.search(r'~max_%s", ([0-9.]+)' % k, src).group(1)) for k in ("vx", "vy", "wz")]
        for i in range(3):
            self.assertLessEqual(DWA["nav_limits"][i], gate[i])


if __name__ == "__main__":
    unittest.main()
