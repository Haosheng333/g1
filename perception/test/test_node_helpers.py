"""ROS-free tests for the node's helpers: image decoding (replaces cv_bridge)
and the calibration-yaml extrinsics loader. Run from perception/:

    PYTHONPATH=src python3 -m unittest discover -s test -p test_node_helpers.py -v
"""
import os
import sys
import tempfile
import types
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from perception.ros_image import image_to_array  # noqa: E402


def fake_image(arr, encoding, bigendian=False, pad=0):
    h, w = arr.shape[:2]
    rows = arr.reshape(h, -1).view(np.uint8)
    if pad:
        rows = np.concatenate([rows, np.zeros((h, pad), np.uint8)], axis=1)
    return types.SimpleNamespace(encoding=encoding, height=h, width=w, step=rows.shape[1],
                                 is_bigendian=int(bigendian), data=rows.tobytes())


class ImageDecoding(unittest.TestCase):
    def test_bgr8_passthrough(self):
        img = np.random.randint(0, 255, (4, 6, 3), np.uint8)
        np.testing.assert_array_equal(image_to_array(fake_image(img, "bgr8"), 3), img)

    def test_rgb8_is_swapped_to_bgr(self):
        img = np.random.randint(0, 255, (4, 6, 3), np.uint8)
        np.testing.assert_array_equal(image_to_array(fake_image(img, "rgb8"), 3), img[:, :, ::-1])

    def test_row_padding_is_dropped(self):
        img = np.random.randint(0, 255, (3, 5, 3), np.uint8)
        np.testing.assert_array_equal(image_to_array(fake_image(img, "bgr8", pad=7), 3), img)

    def test_depth_16uc1_little_endian_mm(self):
        d = np.random.randint(0, 5000, (4, 6), np.uint16)
        out = image_to_array(fake_image(d, "16UC1"), 1)
        self.assertEqual(out.dtype, np.uint16)
        np.testing.assert_array_equal(out, d)

    def test_depth_big_endian(self):
        d = np.random.randint(0, 5000, (4, 6), np.uint16)
        msg = fake_image(d.astype(">u2"), "16UC1", bigendian=True)
        np.testing.assert_array_equal(image_to_array(msg, 1), d)

    def test_wrong_encoding_rejected(self):
        with self.assertRaises(ValueError):
            image_to_array(fake_image(np.zeros((2, 2), np.uint16), "32FC1"), 1)
        with self.assertRaises(ValueError):
            image_to_array(fake_image(np.zeros((2, 2, 3), np.uint8), "mono8"), 3)

    def test_short_buffer_rejected(self):
        msg = fake_image(np.zeros((2, 2, 3), np.uint8), "bgr8")
        msg.data = msg.data[:-1]
        with self.assertRaises(ValueError):
            image_to_array(msg, 3)


class DepthInBox(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import ultralytics  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest(str(exc))

    def test_median_ignores_holes_and_outliers(self):
        from perception.m1_perception import median_depth_in_box
        d = np.zeros((480, 640))
        d[100:280, 200:300] = 0.43                     # can front, 100x180 box
        d[100:280:7, 200:300] = 0.0                    # dropout rows
        d[185:195, 245:255] = 0.60                     # a 10x10 outlier blob at the centre
        z = median_depth_in_box(d, (200, 100, 300, 280))
        self.assertAlmostEqual(z, 0.43, places=6)

    def test_falls_back_to_centre_patch(self):
        from perception.m1_perception import median_depth_in_box
        d = np.zeros((480, 640))
        d[190, 250] = 0.5
        self.assertAlmostEqual(median_depth_in_box(d, (200, 100, 300, 280)), 0.5)
        self.assertIsNone(median_depth_in_box(np.zeros((480, 640)), (200, 100, 300, 280)))


class CalibrationYamlLoader(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import yaml  # noqa: F401
            import ultralytics  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest(f"{exc}")

    def _write(self, parent="torso_link", R=None):
        import yaml
        R = np.eye(3) if R is None else R
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = [0.1, 0.2, 0.3]
        path = os.path.join(tempfile.mkdtemp(), "d455_extrinsics_test.yaml")
        with open(path, "w") as f:
            yaml.safe_dump({"parent_to_color_optical": {"parent_frame": parent,
                            "child_frame": "camera_color_optical_frame",
                            "matrix_4x4": M.tolist()}}, f)
        return path

    def test_loads_matrix_block(self):
        from perception.m1_perception import extrinsics_from_calibration_yaml
        ext = extrinsics_from_calibration_yaml(self._write())
        np.testing.assert_allclose(ext.R_base_opt, np.eye(3))
        np.testing.assert_allclose(ext.t_base_opt, [0.1, 0.2, 0.3])

    def test_rejects_wrong_parent_frame(self):
        from perception.m1_perception import extrinsics_from_calibration_yaml
        with self.assertRaises(ValueError):
            extrinsics_from_calibration_yaml(self._write(parent="base_link"))

    def test_rejects_non_rotation(self):
        from perception.m1_perception import extrinsics_from_calibration_yaml
        with self.assertRaises(ValueError):
            extrinsics_from_calibration_yaml(self._write(R=np.diag([1.0, 1.0, 1.01])))

    def test_negative_index_is_refused(self):
        from perception.m1_perception import CameraIntrinsics, Extrinsics, handle_get_object_position

        class NoBoxes:
            boxes = []
            names = {}

        class Model:
            def predict(self, *a, **k):
                return [NoBoxes()]

        K = CameraIntrinsics(600, 600, 320, 240)
        r = handle_get_object_position(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640), np.uint16),
                                       Model(), K, Extrinsics(), label="bottle", index=-1)
        self.assertFalse(r.success)
        self.assertIn("index must be >= 0", r.message)


if __name__ == "__main__":
    unittest.main()
