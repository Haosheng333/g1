"""Offline tests (no ROS master).  Run:  python3 -m unittest test_cloud_utils -v
or via catkin:  catkin_make run_tests_g1_slam"""
import math
import os
import tempfile
import unittest

import numpy as np
from scipy.spatial import cKDTree

from g1_slam import cloud_utils as cu
from g1_slam import grid_utils as gu


def _plane(rng, p0, u, v, nu, nv, spacing=0.05):
    a = rng.uniform(0, nu, int(nu * nv / spacing ** 2 * 0.5))
    b = rng.uniform(0, nv, len(a))
    return p0 + np.outer(a, u) + np.outer(b, v)


def make_room(seed=1):
    """z-DOWN room: floor at z=+1.0, ceiling at z=-1.4, asymmetric furniture."""
    rng = np.random.RandomState(seed)
    ex, ey, ez = np.array([1., 0, 0]), np.array([0, 1., 0]), np.array([0, 0, 1.])
    parts = [
        _plane(rng, np.array([0, 0, 1.0]), ex, ey, 8, 6),          # floor
        _plane(rng, np.array([0, 0, -1.4]), ex, ey, 8, 6),         # ceiling
        _plane(rng, np.array([0, 0, -1.4]), ey, ez, 6, 2.4),       # wall x=0
        _plane(rng, np.array([8, 0, -1.4]), ey, ez, 6, 2.4),       # wall x=8
        _plane(rng, np.array([0, 0, -1.4]), ex, ez, 8, 2.4),       # wall y=0
        _plane(rng, np.array([0, 6, -1.4]), ex, ez, 8, 2.4),       # wall y=6
        _plane(rng, np.array([3, 3, -1.4]), ey, ez, 3, 2.4),       # partition x=3
        _plane(rng, np.array([5, 1, 0.2]), ex, ey, 2, 1),          # box top
        _plane(rng, np.array([5, 1, 0.2]), ey, ez, 1, 0.8),        # box side
    ]
    return np.vstack(parts).astype(np.float32)


class CloudUtilsTest(unittest.TestCase):
    def test_pcd_roundtrip(self):
        pts = np.random.RandomState(0).uniform(-5, 5, (1000, 3)).astype(np.float32)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "a.pcd")
            cu.write_pcd(p, pts)
            back = cu.read_pcd_xyz(p)
        np.testing.assert_allclose(back, pts, atol=1e-6)

    def test_voxel_downsample_unique(self):
        pts = np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [1.0, 1.0, 1.0]], np.float32)
        self.assertEqual(len(cu.voxel_downsample(pts, 0.1)), 2)

    def test_quat_roundtrip(self):
        for rpy in [(0, 0, 0), (math.pi, 0, 0), (0.1, -0.2, 2.5), (math.pi, 0.05, -1.0)]:
            R = cu.rpy_to_R(*rpy)
            R2 = cu.quat_to_R(*cu.R_to_quat(R))
            np.testing.assert_allclose(R, R2, atol=1e-9)

    def test_floor_estimate_z_down(self):
        z = cu.estimate_floor_z_down(make_room())
        self.assertIsNotNone(z)
        self.assertAlmostEqual(z, 1.0, delta=0.06)

    def test_relocalize_recovers_known_pose(self):
        room = make_room()
        tree = cKDTree(cu.voxel_downsample(room, 0.25))
        dst = cu.voxel_downsample(room, 0.25)
        # robot stands at (2, 4.5) in the saved frame; the new session's camera_init
        # sits at the robot start pose rotated 40 deg and tilted 3 deg.
        T_saved_live = cu.make_T(cu.rpy_to_R(0.05, 0.0, math.radians(40)), [2.0, 4.5, 0.0])
        live_all = cu.apply_T(cu.invert_T(T_saved_live), room)
        centre = cu.apply_T(cu.invert_T(T_saved_live), np.array([[2.0, 4.5, 0.0]]))[0]
        near = live_all[np.linalg.norm(live_all[:, :2] - centre[:2], axis=1) < 4.5]
        src = cu.voxel_downsample(near, 0.25)
        T, frac, rmse, runner_up = cu.register(src, dst, tree)
        self.assertGreater(frac, 0.8)
        self.assertGreater(frac - runner_up, 0.03)  # unambiguous
        self.assertLess(rmse, 0.2)
        err = cu.invert_T(T_saved_live) @ T
        self.assertLess(np.linalg.norm(err[:3, 3]), 0.25)
        self.assertLess(abs(cu.yaw_of(err[:3, :3])), math.radians(4))


class GridUtilsTest(unittest.TestCase):
    def test_tiers(self):
        g = np.zeros((40, 40), np.int8)
        g[:, 20] = gu.OBSTACLE
        g[0, 0] = gu.UNKNOWN
        t = gu.compose_tiers(g, 0.05, danger_m=0.25, risk_m=0.50)
        self.assertEqual(t[10, 20], gu.OBSTACLE)
        self.assertEqual(t[10, 18], gu.DANGER)    # 0.10 m from the wall
        self.assertEqual(t[10, 13], gu.RISK)      # 0.35 m
        self.assertEqual(t[10, 2], gu.WALKABLE)   # 0.90 m
        self.assertEqual(t[0, 0], gu.UNKNOWN)

    def test_hole_fill_keeps_frontier_and_obstacles(self):
        g = np.full((30, 30), gu.UNKNOWN, np.int8)
        g[5:25, 5:25] = gu.WALKABLE
        rng = np.random.RandomState(3)
        holes = rng.rand(20, 20) < 0.5            # 50% speckle inside the room
        g[5:25, 5:25][holes] = gu.UNKNOWN
        g[15, 15] = gu.OBSTACLE
        out = gu.fill_unknown_holes(g, 3)
        self.assertEqual(out[15, 15], gu.OBSTACLE)                      # obstacle untouched
        self.assertGreater((out[8:22, 8:22] == gu.WALKABLE).mean(), 0.9)  # interior filled
        self.assertTrue((out[:3, :] == gu.UNKNOWN).all())               # frontier not grown outward
        self.assertTrue((out[:, 27:] == gu.UNKNOWN).all())

    def test_pgm_orientation(self):
        # image row 0 = TOP (max y). One occupied pixel in the top-left corner.
        with tempfile.TemporaryDirectory() as d:
            img = np.full((4, 6), 254, np.uint8)
            img[0, 0] = 0
            with open(os.path.join(d, "m.pgm"), "wb") as f:
                f.write(b"P5\n6 4\n255\n" + img.tobytes())
            with open(os.path.join(d, "m.yaml"), "w") as f:
                f.write("image: m.pgm\nresolution: 0.5\norigin: [-1.0, -2.0, 0.0]\n"
                        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
            grid, res, ox, oy = gu.load_map_yaml(os.path.join(d, "m.yaml"))
        self.assertEqual((res, ox, oy), (0.5, -1.0, -2.0))
        self.assertEqual(grid.shape, (4, 6))
        self.assertEqual(grid[3, 0], gu.OBSTACLE)  # top image row -> LAST grid row
        self.assertEqual(grid[0, 0], gu.WALKABLE)


if __name__ == "__main__":
    unittest.main()
