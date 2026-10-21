"""Two cans of different size must return different centres: height and radius
are measured from the depth image, not assumed.

Analytic depth render of a vertical cylinder on a table (ray/cylinder,
ray/top-disk, ray/plane), fed through run_detection_pipeline with a fake
detector that returns the silhouette box.

    PYTHONPATH=src python3 -m unittest discover -s test -p test_measured_size.py -v
"""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

W, H = 1280, 720


def _deps():
    try:
        import ultralytics  # noqa: F401
    except ImportError as exc:
        raise unittest.SkipTest(str(exc))


def render(K, ext, cx, cy, r, z_table, height):
    """Depth (m, 0 = none) of table + cylinder, and the cylinder's silhouette box."""
    z_top = z_table + height
    vv, uu = np.mgrid[0:H, 0:W]
    d = np.stack([(uu - K.cx) / K.fx, (vv - K.cy) / K.fy, np.ones((H, W))], axis=-1) @ ext.R_base_opt.T  # z_opt = 1 per unit t
    cam = ext.t_base_opt
    big = 1e9
    # table plane
    t_tab = np.where(d[..., 2] < 0, (z_table - cam[2]) / np.where(d[..., 2] < 0, d[..., 2], 1), big)
    # cylinder side: |cam_xy + t d_xy - c|^2 = r^2
    ox, oy = cam[0] - cx, cam[1] - cy
    a = d[..., 0] ** 2 + d[..., 1] ** 2
    b = 2 * (ox * d[..., 0] + oy * d[..., 1])
    c = ox * ox + oy * oy - r * r
    disc = b * b - 4 * a * c
    t_side = np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), big)
    z_side = cam[2] + t_side * d[..., 2]
    t_side = np.where((t_side > 0) & (z_side >= z_table) & (z_side <= z_top), t_side, big)
    # top disk
    t_topd = np.where(d[..., 2] < 0, (z_top - cam[2]) / np.where(d[..., 2] < 0, d[..., 2], 1), big)
    px, py = cam[0] + t_topd * d[..., 0] - cx, cam[1] + t_topd * d[..., 1] - cy
    t_topd = np.where((t_topd > 0) & (px * px + py * py <= r * r), t_topd, big)
    t_cyl = np.minimum(t_side, t_topd)
    t = np.minimum(t_tab, t_cyl)
    depth = np.where(t < big, t, 0.0)
    mask = t_cyl < t_tab
    ys, xs = np.where(mask)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
    return depth, bbox


class _T:
    def __init__(self, v): self.v = np.asarray(v, float)
    def item(self): return float(self.v)
    def cpu(self): return self
    def numpy(self): return self.v


def fake_model(bbox):
    class Box:
        cls, conf, xyxy = _T(0), _T(0.9), [_T(bbox)]

    class Res:
        boxes, names = [Box()], {0: "bottle"}

    class Model:
        def predict(self, *a, **k): return [Res()]
    return Model()


class MeasuredSize(unittest.TestCase):
    def setUp(self):
        _deps()
        from perception.m1_perception import CameraIntrinsics, extrinsics_from_mount
        self.K = CameraIntrinsics(fx=645.0, fy=644.0, cx=640.0, cy=360.0)
        self.ext = extrinsics_from_mount((0.11, 0.0, 0.41), (0.0, math.radians(68), 0.0))

    def _run(self, cx, cy, r, z_table, height):
        from perception.m1_perception import run_detection_pipeline
        depth, bbox = render(self.K, self.ext, cx, cy, r, z_table, height)
        depth_mm = np.round(depth * 1000).astype(np.uint16)
        dets = run_detection_pipeline(np.zeros((H, W, 3), np.uint8), depth_mm, fake_model(bbox), self.K, self.ext)
        self.assertEqual(len(dets), 1)
        return dets[0], np.array([cx, cy, z_table + height / 2])

    def test_two_cans_different_heights(self):
        results = {}
        for height in (0.10, 0.14, 0.17):
            d, centre = self._run(0.42, 0.05, 0.029, -0.05, height)
            self.assertIsNotNone(d.height_m, "height should be measured, not assumed")
            self.assertAlmostEqual(d.height_m, height, delta=0.006, msg=f"height {d.height_m} vs {height}")
            err = np.linalg.norm(d.p_base - centre)
            self.assertLess(err, 0.005, f"centre error {err*100:.2f} cm for a {height} m can")
            results[height] = d.p_base[2]
        self.assertGreater(results[0.17] - results[0.10], 0.03)

    def test_radius_is_measured(self):
        for r in (0.025, 0.033, 0.045):
            d, centre = self._run(0.45, -0.08, r, -0.03, 0.12)
            self.assertIsNotNone(d.radius_m)
            self.assertAlmostEqual(d.radius_m, r, delta=0.004, msg=f"radius {d.radius_m} vs {r}")
            self.assertLess(np.linalg.norm(d.p_base - centre), 0.005)

    def test_fallback_when_top_is_missing(self):
        from perception.m1_perception import run_detection_pipeline
        depth, bbox = render(self.K, self.ext, 0.42, 0.0, 0.029, -0.05, 0.14)
        depth_mm = np.round(depth * 1000).astype(np.uint16)
        x1, y1, x2, y2 = bbox
        depth_mm[y1:y1 + int(0.6 * (y2 - y1)), :] = 0  # no depth on the upper part of the can
        d = run_detection_pipeline(np.zeros((H, W, 3), np.uint8), depth_mm, fake_model(bbox), self.K, self.ext)[0]
        self.assertIsNone(d.height_m)  # fell back to CLASS_HEIGHT_M
        self.assertAlmostEqual(d.p_base[2], -0.05 + 0.07, delta=0.006)


if __name__ == "__main__":
    unittest.main()
