"""The returned point for an upright can is its geometric centre.

Synthetic scene: a camera looking down at a cylinder of known radius/height
standing on a table. The front-surface sample and the box's bottom edge are
projected analytically; the recovered centre must match the true one.

    PYTHONPATH=src python3 -m unittest discover -s test -p test_cylinder_center.py -v
"""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, HERE)


def _deps():
    try:
        import ultralytics  # noqa: F401
    except ImportError as exc:
        raise unittest.SkipTest(str(exc))


class UprightCylinder(unittest.TestCase):
    def setUp(self):
        _deps()
        from perception.m1_perception import CameraIntrinsics, extrinsics_from_mount
        self.m = sys.modules["perception.m1_perception"]
        self.K = CameraIntrinsics(fx=645.0, fy=644.0, cx=640.0, cy=360.0)
        # camera 0.41 m up, 0.11 m forward, pitched 60 deg down (D455-like)
        self.ext = extrinsics_from_mount((0.11, 0.0, 0.41), (0.0, math.radians(60), 0.0))
        self.r, self.h = 0.033, 0.14

    def _project(self, p_base):
        p_opt = self.ext.R_base_opt.T @ (p_base - self.ext.t_base_opt)
        u = self.K.cx + p_opt[0] * self.K.fx / p_opt[2]
        v = self.K.cy + p_opt[1] * self.K.fy / p_opt[2]
        return u, v, p_opt[2]

    def _scene(self, cx, cy, z_table, sample_frac=0.65):
        """True centre, and the (bbox, surface sample) a detector + depth would give.

        The box is the real silhouette: extreme rows/columns of the top and
        bottom circles. The depth sample is the front surface part-way up
        (the top face is visible, so the box centre is above mid-height).
        """
        centre = np.array([cx, cy, z_table + self.h / 2])
        cam = self.ext.t_base_opt
        to_cam = np.array([cam[0] - cx, cam[1] - cy, 0.0]); to_cam /= np.linalg.norm(to_cam)
        front = centre[:2] + self.r * to_cam[:2]
        p_sample = np.array([front[0], front[1], z_table + sample_frac * self.h])
        az = np.linspace(0, 2 * math.pi, 360, endpoint=False)
        us, vs = [], []
        for z in (z_table, z_table + self.h):
            for a in az:
                u, v, _ = self._project(np.array([cx + self.r * math.cos(a), cy + self.r * math.sin(a), z]))
                us.append(u); vs.append(v)
        bbox = (int(round(min(us))), int(round(min(vs))), int(round(max(us))), int(round(max(vs))))
        return centre, bbox, p_sample

    def test_recovers_centre_on_a_table(self):
        for cx, cy, zt in ((0.40, 0.0, -0.035), (0.55, 0.12, -0.05), (0.35, -0.10, 0.0)):
            centre, bbox, p_sample = self._scene(cx, cy, zt)
            got = self.m.upright_cylinder_center(p_sample, bbox, self.K, self.ext, self.r, self.h, 720)
            self.assertIsNotNone(got)
            err = np.linalg.norm(got - centre)
            self.assertLess(err, 0.004, f"centre error {err*100:.2f} cm at {centre}")

    def test_plain_ray_push_is_worse_from_above(self):
        """Documents why the cylinder model exists: the spec's ray push misses by cm."""
        centre, bbox, p_sample = self._scene(0.40, 0.0, -0.035)
        p_opt = self.ext.R_base_opt.T @ (p_sample - self.ext.t_base_opt)
        plain = self.m.optical_to_base(self.m.push_along_ray(p_opt, self.r), self.ext)
        self.assertGreater(np.linalg.norm(plain - centre), 0.01)

    def test_falls_back_when_bottom_edge_is_clipped(self):
        centre, bbox, p_sample = self._scene(0.40, 0.0, -0.035)
        x1, y1, x2, y2 = bbox
        self.assertIsNone(self.m.upright_cylinder_center(p_sample, (x1, y1, x2, 719), self.K, self.ext, self.r, self.h, 720))

    def test_pipeline_uses_it_for_bottle(self):
        """End to end through run_detection_pipeline on an analytic depth render."""
        from perception.m1_perception import run_detection_pipeline
        from test_measured_size import fake_model, render
        from perception.m1_perception import extrinsics_from_mount
        ext = extrinsics_from_mount((0.11, 0.0, 0.41), (0.0, math.radians(68), 0.0))
        depth, bbox = render(self.K, ext, 0.40, 0.0, 0.033, -0.035, 0.14)
        depth_mm = np.round(depth * 1000).astype(np.uint16)
        dets = run_detection_pipeline(np.zeros((720, 1280, 3), np.uint8), depth_mm, fake_model(bbox), self.K, ext)
        self.assertEqual(len(dets), 1)
        err = np.linalg.norm(dets[0].p_base - np.array([0.40, 0.0, 0.035]))
        self.assertLess(err, 0.005, f"pipeline centre error {err*100:.2f} cm")

    def test_clipped_box_falls_back_to_constants(self):
        from perception.m1_perception import run_detection_pipeline, CLASS_HEIGHT_M
        from test_measured_size import fake_model, render
        from perception.m1_perception import extrinsics_from_mount
        ext = extrinsics_from_mount((0.11, 0.0, 0.41), (0.0, math.radians(68), 0.0))
        depth, bbox = render(self.K, ext, 0.60, 0.15, 0.033, -0.05, 0.14)
        self.assertEqual(bbox[1], 0, "scene should clip the can at the top of the image")
        d = run_detection_pipeline(np.zeros((720, 1280, 3), np.uint8), np.round(depth * 1000).astype(np.uint16),
                                   fake_model(bbox), self.K, ext)[0]
        self.assertIsNone(d.height_m)
        self.assertIsNone(d.radius_m)


class TableFromDepth(unittest.TestCase):
    def setUp(self):
        _deps()
        from perception.m1_perception import CameraIntrinsics, extrinsics_from_mount, table_height_below_box
        self.f = table_height_below_box
        self.K = CameraIntrinsics(fx=645.0, fy=644.0, cx=640.0, cy=360.0)
        self.ext = extrinsics_from_mount((0.11, 0.0, 0.41), (0.0, math.radians(60), 0.0))

    def _table_depth(self, z_table):
        """Depth image of a horizontal plane at z_table, in metres."""
        vv, uu = np.mgrid[0:720, 0:1280]
        d = self.ext.R_base_opt @ np.stack([(uu - self.K.cx) / self.K.fx, (vv - self.K.cy) / self.K.fy, np.ones_like(uu, float)])\
            .reshape(3, -1)
        t = (z_table - self.ext.t_base_opt[2]) / d[2]  # ray parameter where the ray hits the plane
        depth = np.where(t > 0, t, 0.0).reshape(720, 1280)  # z_opt == t for a unit-z_opt ray
        return depth

    def test_reads_plane_height(self):
        depth = self._table_depth(-0.05)
        z = self.f(depth, (560, 200, 640, 380), self.K, self.ext)
        self.assertIsNotNone(z)
        self.assertAlmostEqual(z, -0.05, delta=0.002)

    def test_rejects_holes_and_non_flat(self):
        depth = self._table_depth(-0.05)
        depth[383:395, :] = 0.0
        self.assertIsNone(self.f(depth, (560, 200, 640, 380), self.K, self.ext))
        depth = self._table_depth(-0.05)
        depth[383:395, 600:640] *= 1.2  # a step: not flat
        self.assertIsNone(self.f(depth, (560, 200, 640, 380), self.K, self.ext))

    def test_pipeline_prefers_depth_table(self):
        from perception.m1_perception import upright_cylinder_center
        p_sample = np.array([0.40, 0.0, 0.03])
        got = upright_cylinder_center(p_sample, (560, 200, 640, 380), self.K, self.ext, 0.033, 0.14, 720, z_table=-0.05)
        self.assertAlmostEqual(got[2], 0.02, places=9)


if __name__ == "__main__":
    unittest.main()
