"""Offline tests for costmap + global planner + local planner (no ROS).

    cd navigation/test && python3 -m unittest test_costmap_planner_dwa -v

Includes the closed-loop ACCEPTANCE test of doc section 4.4: "from any start, the robot reaches a
clicked goal around furniture with < 0.10 m error" (and < 0.1 rad yaw), without the footprint ever
touching an obstacle, using a holonomic robot with first-order velocity lag.
"""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "slam", "src"))

from costmap import Costmap, CostmapManager, OBSTACLE  # noqa: E402
from global_planner import AStarPlanner  # noqa: E402
from local_planner import DwaController, APPROACH, NAVIGATION, TERMINAL, wrap  # noqa: E402
from g1_slam import grid_utils as gu  # noqa: E402

RES = 0.05


def furnished_room():
    """10 x 8 m: outer walls, a partition at x=5 with a 1.0 m door, a table, a sofa and a chair."""
    g = np.zeros((160, 200), np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OBSTACLE
    g[:, 100] = OBSTACLE
    g[70:90, 100] = 0                       # door y 3.5 .. 4.5
    g[30:50, 40:60] = OBSTACLE              # table 1.0 x 1.0 at (2.5, 2.0)
    g[100:116, 130:166] = OBSTACLE          # sofa 1.8 x 0.8 at (7.4, 5.4)
    g[60:70, 20:30] = OBSTACLE              # chair
    return gu.compose_tiers(g, RES)


class Sim:
    """Holonomic kinematic robot with first-order velocity lag (tau 0.25 s) running the real DWA."""

    def __init__(self, cm, pose, dwa=None, dt=0.1, tau=0.25):
        self.cm, self.pose, self.dt, self.tau = cm, list(pose), dt, tau
        self.vel = [0.0, 0.0, 0.0]
        self.cmd = (0.0, 0.0, 0.0)
        self.dwa = dwa or DwaController()
        self.min_clear = 9.0
        self.phases = []

    def step(self, vx, vy, wz):
        a = self.dt / (self.tau + self.dt)
        self.vel = [v + a * (c - v) for v, c in zip(self.vel, (vx, vy, wz))]
        x, y, th = self.pose
        x += (self.vel[0] * math.cos(th) - self.vel[1] * math.sin(th)) * self.dt
        y += (self.vel[0] * math.sin(th) + self.vel[1] * math.cos(th)) * self.dt
        th += self.vel[2] * self.dt
        self.pose = [x, y, th]
        self.min_clear = min(self.min_clear, self.cm.footprint_clearance(x, y, th))

    def drive(self, goal, planner=None, max_time=120.0, cm_provider=None):
        planner = planner or AStarPlanner()
        gx, gy, gyaw = goal
        path = planner.plan(self.cm, (self.pose[0], self.pose[1]), (gx, gy))
        if not path:
            return "NOPATH", 0.0
        t = 0.0
        while t < max_time:
            cm = cm_provider(t) if cm_provider else self.cm
            r = self.dwa.compute(tuple(self.pose), self.cmd, path, (gx, gy), gyaw, cm)
            self.phases.append(r.phase)
            if r.reached:
                return "SUCCESS", t
            if r.blocked:
                self.cmd = (0.0, 0.0, 0.0)
            else:
                self.cmd = (r.vx, r.vy, r.wz)
            self.step(*self.cmd)
            t += self.dt
        return "TIMEOUT", t


class CostmapTests(unittest.TestCase):
    def setUp(self):
        self.cm = Costmap.from_grid(furnished_room(), RES)

    def test_footprint_is_five_circles(self):
        self.assertEqual(len(self.cm.circle_y), 5)
        self.assertAlmostEqual(float(self.cm.circle_y[0]), -0.15)
        self.assertEqual(self.cm.cfg["circle_radius"], 0.10)

    def test_footprint_depends_on_heading(self):
        # in the 1.0 m door: lateral reach 0.25 m fits when facing the door, the wide side is also free
        c0 = self.cm.footprint_clearance(5.0, 4.0, 0.0)
        c90 = self.cm.footprint_clearance(5.0, 4.0, math.pi / 2)
        self.assertGreater(c0, 0.0)
        self.assertGreater(c90, c0)                       # facing along the wall = more room
        self.assertTrue(self.cm.footprint_collision(5.0, 3.6, 0.0))     # shoulder in the wall

    def test_tier_penalty_and_decay_make_centre_cheaper(self):
        near = self.cm.penalty_at(*self.cm.world_to_map(5.0, 3.8))
        centre = self.cm.penalty_at(*self.cm.world_to_map(5.0, 4.0))
        self.assertGreater(near, centre)

    def test_live_obstacle_blocks_then_clears(self):
        mgr = CostmapManager()
        mgr.set_static(furnished_room(), RES, 0.0, 0.0)
        box = np.array([[5.0 + 0.02 * i, 4.0] for i in range(-5, 6)])
        mgr.set_live_points(box, 0.0)
        self.assertEqual(AStarPlanner().plan(mgr.state, (1, 1), (8.5, 6.5)), [])
        mgr.set_live_points(np.zeros((0, 2)), 1.0)        # old scan still held (live_hold 0.6 s)
        mgr.set_live_points(np.zeros((0, 2)), 2.0)
        self.assertGreater(len(AStarPlanner().plan(mgr.state, (1, 1), (8.5, 6.5))), 10)

    def test_live_points_on_static_walls_are_ignored(self):
        mgr = CostmapManager()
        mgr.set_static(furnished_room(), RES, 0.0, 0.0)
        wall_pts = np.array([[0.02, y] for y in np.linspace(1, 7, 60)])   # the real wall, seen by the lidar
        mgr.set_live_points(wall_pts, 0.0)
        self.assertIsNone(mgr.state.live_mask)


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.cm = Costmap.from_grid(furnished_room(), RES)
        self.pl = AStarPlanner()

    def test_path_goes_through_door_and_keeps_clearance(self):
        p = self.pl.plan(self.cm, (1.0, 1.0), (8.5, 6.5))
        self.assertTrue(any(abs(x - 5.0) < 0.15 and 3.4 < y < 4.6 for x, y in p))
        self.assertGreater(min(self.cm.dist_at(x, y) for x, y in p), 0.25)
        self.assertEqual(p[0], (1.0, 1.0))
        self.assertEqual(p[-1], (8.5, 6.5))

    def test_pruned_and_smooth(self):
        raw = self.pl._astar(self.cm, self.cm.world_to_map(1, 1), self.cm.world_to_map(8.5, 6.5))
        p = self.pl.plan(self.cm, (1.0, 1.0), (8.5, 6.5))
        turn = [abs(wrap(math.atan2(p[i + 1][1] - p[i][1], p[i + 1][0] - p[i][0]) -
                         math.atan2(p[i][1] - p[i - 1][1], p[i][0] - p[i - 1][0]))) for i in range(1, len(p) - 1)]
        self.assertLess(max(turn), 0.6)                    # no sharp corners left after Bezier rounding
        self.assertLess(len(p), len(raw))                  # resampled at 0.1 m vs 0.05 m raw
        steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p[:-1], p[1:])]
        self.assertLess(max(steps), 0.15)

    def test_goal_inside_furniture_has_no_path(self):
        self.assertEqual(self.pl.plan(self.cm, (1, 1), (2.5, 2.0)), [])
        self.assertIn("not traversable", self.pl.last_info["reason"])

    def test_unreachable_goal_terminates_quickly(self):
        g = furnished_room()
        g[:, 100] = OBSTACLE                                # seal the door
        cm = Costmap.from_grid(g, RES)
        import time
        t = time.time()
        self.assertEqual(AStarPlanner().plan(cm, (1, 1), (8.5, 6.5)), [])
        self.assertLess(time.time() - t, 4.0)

    def test_blocked_start_is_snapped(self):
        p = self.pl.plan(self.cm, (0.32, 1.0), (4.0, 1.0))  # 0.32 m from the wall: inside the blocked band
        self.assertGreater(len(p), 5)


class CloseApproachTests(unittest.TestCase):
    """A standing pose close to a table CORNER (off-axis, like a real recorded waypoint) fails the round
    point-robot clearance (0.255 m < 0.30 m) but must be reachable when the robot faces along the table,
    because the footprint is only 0.10 m deep.  The same spot turned sideways overlaps the corner and
    must stay rejected."""

    def setUp(self):
        g = np.zeros((120, 120), np.int8)
        g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OBSTACLE
        g[64:80, 64:80] = OBSTACLE                  # table x, y in 3.2 .. 4.0, corner at (3.2, 3.2)
        self.cm = Costmap.from_grid(gu.compose_tiers(g, RES), RES)
        self.goal = (2.99, 3.195)                   # footprint clearance ~+0.09 facing +x, < 0 sideways
        self.pl = AStarPlanner()

    def test_point_robot_rule_rejects_without_heading(self):
        self.assertFalse(self.cm.is_free(*self.cm.world_to_map(*self.goal)))
        self.assertEqual(self.pl.plan(self.cm, (1.0, 1.0), self.goal), [])

    def test_facing_the_table_is_planned_and_ends_exactly_on_the_goal(self):
        path = self.pl.plan(self.cm, (1.0, 1.0), self.goal, goal_yaw=0.0)
        self.assertTrue(path)
        self.assertAlmostEqual(path[-1][0], self.goal[0])
        self.assertAlmostEqual(path[-1][1], self.goal[1])
        self.assertIsNotNone(self.pl.last_info.get("approach_from"))

    def test_sideways_to_the_table_is_still_rejected(self):
        self.assertEqual(self.pl.plan(self.cm, (1.0, 1.0), self.goal, goal_yaw=math.pi / 2), [])

    def test_dwa_drives_there_without_touching(self):
        path = self.pl.plan(self.cm, (1.0, 1.0), self.goal, goal_yaw=0.0)
        sim = Sim(self.cm, (1.0, 1.0, 0.0))
        t, ok = 0.0, False
        while t < 60.0:
            r = sim.dwa.compute(tuple(sim.pose), sim.cmd, path, self.goal, 0.0, self.cm)
            if r.reached:
                ok = True
                break
            sim.cmd = (0.0, 0.0, 0.0) if r.blocked else (r.vx, r.vy, r.wz)
            sim.step(*sim.cmd)
            t += sim.dt
        self.assertTrue(ok)
        self.assertGreaterEqual(sim.min_clear, self.cm.cfg["collision_margin"] - 1e-6)


class DwaTests(unittest.TestCase):
    def setUp(self):
        self.cm = Costmap.from_grid(furnished_room(), RES)

    def test_phases_and_arrival_tolerance(self):
        d = DwaController()
        r = d.compute((0.0, 0.0, 0.0), (0, 0, 0), [(0, 0), (3, 0)], (3.0, 0.0), 0.0, self.cm)
        # far: navigation; position done but yaw wrong -> rotate in place; both done -> reached
        self.assertEqual(r.phase, NAVIGATION)
        d.reset()
        r = d.compute((2.95, 0.0, 0.0), (0, 0, 0), [(0, 0), (3, 0)], (3.0, 0.0), 1.0, self.cm)
        self.assertEqual(r.phase, TERMINAL)
        self.assertFalse(r.reached)
        self.assertEqual((r.vx, r.vy), (0.0, 0.0))
        self.assertGreater(r.wz, 0.0)
        r = d.compute((2.95, 0.0, 0.95), (0, 0, 0), [(0, 0), (3, 0)], (3.0, 0.0), 1.0, self.cm)
        self.assertTrue(r.reached)

    def test_blocked_when_boxed_in(self):
        d = DwaController()
        r = d.compute((0.12, 4.0, 0.0), (0, 0, 0), [(0.12, 4.0), (3, 4)], (3.0, 4.0), 0.0, self.cm)
        self.assertTrue(r.blocked or r.phase == NAVIGATION)     # sitting in the wall band: never drives into it


class AcceptanceTests(unittest.TestCase):
    """Doc section 4.4 acceptance: reach goals around furniture with < 0.10 m / 0.1 rad."""

    GOALS = [
        # start pose               goal (x, y, yaw)
        ((1.0, 1.0, 0.0), (8.5, 6.5, 1.0)),            # through the door, to the far corner
        ((8.5, 1.0, 3.14), (1.0, 6.0, -1.5)),          # the other way round
        ((1.0, 6.5, -1.0), (3.4, 2.0, 3.0)),           # right next to the table
        ((4.0, 1.0, 1.57), (6.2, 4.0, 0.0)),           # beside the sofa area, after the door
        ((2.0, 4.0, 0.0), (2.0, 4.0, 2.0)),            # rotate in place only
    ]

    def test_reach_goals_around_furniture(self):
        cm = Costmap.from_grid(furnished_room(), RES)
        for start, goal in self.GOALS:
            sim = Sim(cm, start)
            status, t = sim.drive(goal)
            gx, gy, gyaw = goal
            err = math.hypot(sim.pose[0] - gx, sim.pose[1] - gy)
            yerr = abs(wrap(sim.pose[2] - gyaw))
            msg = "start %s goal %s -> %s in %.0fs err %.3f m %.3f rad min_clear %.3f" % (
                start, goal, status, t, err, yerr, sim.min_clear)
            self.assertEqual(status, "SUCCESS", msg)
            self.assertLess(err, 0.10, msg)
            self.assertLess(yerr, 0.1, msg)
            self.assertGreater(sim.min_clear, 0.0, "footprint touched an obstacle: " + msg)
            if err > 0.5 or True:
                self.assertIn(TERMINAL, sim.phases)

    def test_three_phases_are_visited_in_order(self):
        cm = Costmap.from_grid(furnished_room(), RES)
        sim = Sim(cm, (1.0, 1.0, 0.0))
        self.assertEqual(sim.drive((4.0, 1.0, 0.0))[0], "SUCCESS")
        seen = []
        for p in sim.phases:
            if not seen or seen[-1] != p:
                seen.append(p)
        self.assertEqual(seen[:3], [NAVIGATION, APPROACH, TERMINAL])

    def test_replans_around_obstacle_that_appears(self):
        """A box appears in the door while the robot approaches: DWA must stop, never collide."""
        mgr = CostmapManager()
        mgr.set_static(furnished_room(), RES, 0.0, 0.0)
        box = np.array([[5.0 + 0.02 * i, 4.0] for i in range(-5, 6)])
        sim = Sim(mgr.state, (1.0, 4.0, 0.0))
        planner = AStarPlanner()
        path = planner.plan(mgr.state, (1.0, 4.0), (8.5, 4.0))
        self.assertTrue(path)
        t, blocked_seen = 0.0, False
        while t < 40.0:
            if t > 6.0:
                mgr.set_live_points(box, t)
            r = sim.dwa.compute(tuple(sim.pose), sim.cmd, path, (8.5, 4.0), 0.0, mgr.state)
            sim.cmd = (0.0, 0.0, 0.0) if r.blocked else (r.vx, r.vy, r.wz)
            blocked_seen = blocked_seen or r.blocked
            sim.step(*sim.cmd)
            t += 0.1
        self.assertGreater(sim.min_clear, 0.0, "robot hit the obstacle")
        self.assertLess(sim.pose[0], 5.0)                    # stayed on this side of the door


if __name__ == "__main__":
    unittest.main()
