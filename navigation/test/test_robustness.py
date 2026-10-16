"""Robustness regression tests (no ROS): measurement noise, actuator dead-band, footprint clearance.

    cd navigation/test && python3 -m unittest test_robustness -v

Random furnished rooms, closed loop with first-order velocity lag; collisions are judged against the
GROUND-TRUTH occupancy (not the planner's costmap).  Pins the review fixes: bilinear distance
sampling, half-cell surface offset, path-following cost, 0.30 m planner clearance.
"""
import math
import os
import random
import sys
import unittest

import numpy as np
from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "slam", "src"))

from costmap import Costmap, OBSTACLE  # noqa: E402
from global_planner import AStarPlanner  # noqa: E402
from local_planner import DwaController, wrap  # noqa: E402
from g1_slam import grid_utils as gu  # noqa: E402

RES = 0.05


def random_room(seed, W=160, H=120, n_boxes=6):
    rng = random.Random(seed)
    g = np.zeros((H, W), np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OBSTACLE
    for _ in range(n_boxes):
        w, h = rng.randint(6, 24), rng.randint(6, 24)
        x, y = rng.randint(10, W - 30), rng.randint(10, H - 30)
        g[y:y + h, x:x + w] = OBSTACLE
    return g


def footprint_gap(dist, x, y, th):
    """Smallest (distance to obstacle - 0.10) over the 5 footprint circles; < 0 means touching."""
    s, c = math.sin(th), math.cos(th)
    worst = 9.0
    for yi in np.linspace(-0.2, 0.2, 5):
        mx, my = int((x - yi * s) / RES), int((y + yi * c) / RES)
        if not (0 <= mx < dist.shape[1] and 0 <= my < dist.shape[0]):
            return -1.0
        worst = min(worst, dist[my, mx] - 0.10)
    return worst


def run(seed, noise=0.0, deadband=0.0, gain=1.0, tau=0.25, tmax=240.0):
    raw = random_room(seed)
    dist = distance_transform_edt(raw != OBSTACLE) * RES
    cm = Costmap.from_grid(gu.compose_tiers(raw, RES), RES)
    rng = random.Random(seed * 7 + 1)

    def sample():
        for _ in range(500):
            x, y = rng.uniform(1, 7.5), rng.uniform(1, 5.5)
            if cm.standable(*cm.world_to_map(x, y)):
                return x, y
        return None

    s, g = sample(), sample()
    if s is None or g is None:
        return None
    gyaw, syaw = rng.uniform(-math.pi, math.pi), rng.uniform(-math.pi, math.pi)
    path = AStarPlanner().plan(cm, s, g)
    if not path:
        return None
    dwa = DwaController()
    pose, vel, cur = [s[0], s[1], syaw], [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)
    nrng = np.random.RandomState(seed)
    dt, collisions, reached = 0.1, 0, False
    for _ in range(int(tmax / dt)):
        meas = (pose[0] + nrng.randn() * noise, pose[1] + nrng.randn() * noise,
                pose[2] + nrng.randn() * noise * 0.5)
        res = dwa.compute(meas, cur, path, g, gyaw, cm)
        if res.reached:
            reached = True
            break
        cur = (0.0, 0.0, 0.0) if res.blocked else (res.vx, res.vy, res.wz)
        eff = [0.0 if (abs(c) < deadband and i < 2) else c * gain for i, c in enumerate(cur)]
        a = dt / (tau + dt)
        vel = [v + a * (c - v) for v, c in zip(vel, eff)]
        pose[0] += (vel[0] * math.cos(pose[2]) - vel[1] * math.sin(pose[2])) * dt
        pose[1] += (vel[0] * math.sin(pose[2]) + vel[1] * math.cos(pose[2])) * dt
        pose[2] += vel[2] * dt
        if footprint_gap(dist, *pose) < -0.025:     # half a cell of grid discretisation
            collisions += 1
    err = math.hypot(pose[0] - g[0], pose[1] - g[1])
    yerr = abs(wrap(pose[2] - gyaw))
    return dict(reached=reached, collisions=collisions, err=err, yerr=yerr)


class TestClearance(unittest.TestCase):
    def test_clearance_is_bilinear_and_offset_by_half_cell(self):
        g = np.zeros((60, 60), np.int8)
        g[:, 40:] = OBSTACLE
        cm = Costmap.from_grid(gu.compose_tiers(g, RES), RES)
        # centre of a 5-circle footprint at x = 1.5 m: wall face at x = 2.0 m
        x, y = 1.5, 1.5
        c = float(np.asarray(cm.footprint_clearance_many(np.array([x]), np.array([y]),
                                                         np.array([0.0]))).ravel()[0])
        # nearest circle is 0.5 m from the wall face; minus half cell and circle radius
        self.assertAlmostEqual(c, 0.5 - 0.5 * RES - 0.10, delta=0.03)

    def test_clearance_is_smooth_between_cells(self):
        g = np.zeros((60, 60), np.int8)
        g[:, 40:] = OBSTACLE
        cm = Costmap.from_grid(gu.compose_tiers(g, RES), RES)
        xs = np.linspace(1.50, 1.60, 21)
        d = np.asarray(cm.dist_many(xs, np.full_like(xs, 1.5))).ravel()
        steps = np.abs(np.diff(d))
        self.assertLess(steps.max(), 0.01)          # no cell-sized jumps


class TestClosedLoop(unittest.TestCase):
    N = 12

    def _check(self, **kw):
        runs = [r for r in (run(sd, **kw) for sd in range(self.N)) if r]
        self.assertGreaterEqual(len(runs), self.N - 3)
        for r in runs:
            self.assertEqual(r["collisions"], 0, r)
            self.assertTrue(r["reached"], r)
            self.assertLess(r["err"], 0.10, r)
            self.assertLess(r["yerr"], 0.10, r)

    def test_ideal(self):
        self._check()

    def test_localisation_noise_1cm(self):
        self._check(noise=0.01)

    def test_actuator_deadband(self):
        self._check(deadband=0.03)


if __name__ == "__main__":
    unittest.main()
