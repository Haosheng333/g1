"""The trained checkpoints must not silently disagree with the live tool point.

Each checkpoint embeds the `arm_kinematics.yaml` it was trained against. Nothing
in the pipeline compared the two, so editing `grasp_xyz` redefined the frame the
network was fitted to with no signal at all. These tests exercise the guard that
now detects it, and record the CURRENT known-stale state of the shipped files.

Torch-free by construction: the guard reads the checkpoint as a zip and stubs
the torch classes, so these run on the bare host and in the ROS container.
"""
import io
import os
import pickle
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from arm_planning import checkpoint_meta as cm  # noqa: E402

PKG = Path(os.path.dirname(os.path.abspath(__file__))).parent
CONFIG = PKG / "config" / "arm_kinematics.yaml"
MODELS = PKG / "models"


def _live_config():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def _fake_checkpoint(path, config, validation=0.004, inner="ckpt"):
    """Write a minimal .pt-shaped zip whose data.pkl holds a plain payload."""
    payload = {"arm": config.get("_arm", "left"), "config": config,
               "best_validation_position_mean_m": validation}
    buf = io.BytesIO()
    pickle.dump(payload, buf, protocol=2)
    with zipfile.ZipFile(str(path), "w") as z:
        z.writestr("%s/data.pkl" % inner, buf.getvalue())
    return path


class GuardLogic(unittest.TestCase):
    """Synthetic, deterministic: the guard's own behaviour."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(str(self.tmp)))

    def test_identical_tool_point_is_consistent(self):
        cfg = {"left": {"grasp_xyz": [0.215, 0.06, 0.025], "grasp_rpy": [0.0, 0.0, 0.0]}}
        ck = _fake_checkpoint(self.tmp / "g1_left_hybrid_best.pt", cfg)
        result = cm.compare_tool_point(cfg, ck, "left")
        self.assertTrue(result.consistent, result.describe())
        self.assertEqual(result.differences, {})

    def test_changed_translation_is_detected_with_its_magnitude(self):
        trained = {"left": {"grasp_xyz": [0.215, 0.006, 0.025], "grasp_rpy": [0.0, 0.0, 0.0]}}
        live = {"left": {"grasp_xyz": [0.215, 0.06, 0.025], "grasp_rpy": [0.0, 0.0, 0.0]}}
        ck = _fake_checkpoint(self.tmp / "g1_left_hybrid_best.pt", trained)
        result = cm.compare_tool_point(live, ck, "left")
        self.assertFalse(result.consistent)
        self.assertIn("grasp_xyz", result.differences)
        self.assertAlmostEqual(result.translation_delta_m, 0.054, places=6)

    def test_changed_orientation_is_detected(self):
        trained = {"left": {"grasp_xyz": [0.215, 0.06, 0.025], "grasp_rpy": [0.0, 0.0, 0.0]}}
        live = {"left": {"grasp_xyz": [0.215, 0.06, 0.025], "grasp_rpy": [0.0, 0.0, 0.5]}}
        ck = _fake_checkpoint(self.tmp / "g1_left_hybrid_best.pt", trained)
        result = cm.compare_tool_point(live, ck, "left")
        self.assertIn("grasp_rpy", result.differences)

    def test_tiny_float_noise_is_not_a_difference(self):
        trained = {"left": {"grasp_xyz": [0.215, 0.06, 0.025], "grasp_rpy": [0.0, 0.0, 0.0]}}
        live = {"left": {"grasp_xyz": [0.215, 0.06 + 1e-12, 0.025], "grasp_rpy": [0.0, 0.0, 0.0]}}
        ck = _fake_checkpoint(self.tmp / "g1_left_hybrid_best.pt", trained)
        self.assertTrue(cm.compare_tool_point(live, ck, "left").consistent)

    def test_missing_checkpoint_reports_an_error_not_a_pass(self):
        cfg = {"left": {"grasp_xyz": [0.215, 0.06, 0.025], "grasp_rpy": [0.0, 0.0, 0.0]}}
        result = cm.compare_tool_point(cfg, self.tmp / "absent.pt", "left")
        self.assertFalse(result.consistent)
        self.assertTrue(result.error)

    def test_checkpoint_without_embedded_config_is_an_error(self):
        buf = io.BytesIO()
        pickle.dump({"arm": "left"}, buf, protocol=2)
        path = self.tmp / "g1_left_hybrid_best.pt"
        with zipfile.ZipFile(str(path), "w") as z:
            z.writestr("ckpt/data.pkl", buf.getvalue())
        result = cm.compare_tool_point({"left": {}}, path, "left")
        self.assertFalse(result.consistent)
        self.assertIn("config", result.error)

    def test_corrupt_file_is_an_error_not_a_pass(self):
        path = self.tmp / "g1_left_hybrid_best.pt"
        path.write_bytes(b"not a zip")
        result = cm.compare_tool_point({"left": {}}, path, "left")
        self.assertFalse(result.consistent)
        self.assertTrue(result.error)


class ShippedCheckpoints(unittest.TestCase):
    """The real files in this package, as they stand today."""

    def test_checkpoints_embed_a_readable_tool_point(self):
        """Whatever the verdict, the guard must be able to read these files."""
        for arm in ("left", "right"):
            path = MODELS / (cm.CHECKPOINT_NAME % arm)
            if not path.is_file():
                self.skipTest("checkpoint %s is not present" % (path,))
            embedded = cm.embedded_config(path)
            self.assertIn(arm, embedded)
            self.assertIn("grasp_xyz", embedded[arm])

    def test_shipped_checkpoints_are_stale_after_the_tool_point_correction(self):
        """DELIBERATELY asserts staleness -- it is the current, documented state.

        The 2026-10-14 correction (y 0.006 -> 0.06) moved the tool point 54 mm,
        12-13x each checkpoint's own recorded validation error, so the shipped
        weights no longer describe the configured frame. Retraining is the only
        thing that makes them agree; when it happens THIS TEST FAILS, which is
        the intended signal to flip it to an equality assertion. Do not "fix"
        it by reverting the config.
        """
        for arm in ("left", "right"):
            path = MODELS / (cm.CHECKPOINT_NAME % arm)
            if not path.is_file():
                self.skipTest("checkpoint %s is not present" % (path,))
        comparisons = cm.compare_all(_live_config(), MODELS)
        self.assertEqual(cm.stale_arms(comparisons), ["left", "right"],
                         "; ".join(c.describe() for c in comparisons))
        for c in comparisons:
            self.assertAlmostEqual(c.translation_delta_m, 0.054, places=4, msg=c.describe())
            self.assertGreater(c.translation_delta_m, 10 * c.recorded_validation_m, c.describe())

    def test_live_config_keeps_the_y_negation_mirror_convention(self):
        """right.grasp_xyz must stay left.grasp_xyz with y negated."""
        cfg = _live_config()
        left, right = cfg["left"]["grasp_xyz"], cfg["right"]["grasp_xyz"]
        self.assertAlmostEqual(left[0], right[0], places=9)
        self.assertAlmostEqual(left[1], -right[1], places=9)
        self.assertAlmostEqual(left[2], right[2], places=9)

    def test_corrected_offset_is_recorded(self):
        """Guards against a silent revert to the factor-of-ten-smaller value."""
        cfg = _live_config()
        self.assertAlmostEqual(cfg["left"]["grasp_xyz"][1], 0.06, places=9)
        self.assertAlmostEqual(cfg["right"]["grasp_xyz"][1], -0.06, places=9)


if __name__ == "__main__":
    unittest.main()
