"""ROS-free tests for the viewer overlay and the include_invalid_depth path.

    PYTHONPATH=src python3 -m unittest discover -s test -p test_overlay.py -v
"""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))


def _deps():
    try:
        import cv2  # noqa: F401
        import ultralytics  # noqa: F401
    except ImportError as exc:
        raise unittest.SkipTest(str(exc))


class _Tensor:
    def __init__(self, v):
        self.v = np.asarray(v, dtype=float)

    def item(self):
        return float(self.v)

    def cpu(self):
        return self

    def numpy(self):
        return self.v


class _Box:
    def __init__(self, cls, conf, xyxy):
        self.cls, self.conf, self.xyxy = _Tensor(cls), _Tensor(conf), [_Tensor(xyxy)]


class _Result:
    def __init__(self, boxes, names):
        self.boxes, self.names = boxes, names


class _Model:
    """Two boxes: one over valid depth, one over a depth hole."""
    names = {0: "bottle", 1: "cup"}

    def predict(self, *a, **k):
        return [_Result([_Box(0, 0.9, [100, 100, 140, 180]), _Box(1, 0.6, [300, 300, 340, 360])], self.names)]


class Overlay(unittest.TestCase):
    def setUp(self):
        _deps()
        from perception.m1_perception import CameraIntrinsics, Extrinsics, run_detection_pipeline
        self.run = run_detection_pipeline
        self.K = CameraIntrinsics(600, 600, 320, 240)
        self.ext = Extrinsics()
        self.rgb = np.zeros((480, 640, 3), np.uint8)
        self.depth = np.zeros((480, 640), np.uint16)
        self.depth[100:180, 100:140] = 900  # only the bottle has depth

    def test_invalid_depth_dropped_by_default(self):
        dets = self.run(self.rgb, self.depth, _Model(), self.K, self.ext)
        self.assertEqual([d.label for d in dets], ["bottle"])
        self.assertEqual(dets[0].bbox, (100, 100, 140, 180))
        self.assertEqual(dets[0].uv, (120, 160))  # cylinder classes sample the lower front face
        self.assertAlmostEqual(dets[0].depth_m, 0.9)

    def test_invalid_depth_kept_for_visualisation(self):
        dets = self.run(self.rgb, self.depth, _Model(), self.K, self.ext, include_invalid_depth=True)
        self.assertEqual([d.label for d in dets], ["bottle", "cup"])
        self.assertIsNone(dets[1].p_base)
        self.assertIsNone(dets[1].depth_m)

    def test_draw_and_side_by_side_shapes(self):
        from perception.overlay import draw_detections, side_by_side
        dets = self.run(self.rgb, self.depth, _Model(), self.K, self.ext, include_invalid_depth=True)
        out = draw_detections(self.rgb, dets, target_label="bottle", header="hdr")
        self.assertEqual(out.shape, self.rgb.shape)
        self.assertGreater(int(out.sum()), 0, "nothing was drawn")
        self.assertFalse(np.shares_memory(out, self.rgb))
        both = side_by_side(out, self.depth, dets)
        self.assertEqual(both.shape, (480, 1280, 3))
        # the depth hole stays black on the right half
        self.assertEqual(int(both[310, 640 + 310].sum()), 0)  # inside the box, off its border and marker


if __name__ == "__main__":
    unittest.main()
