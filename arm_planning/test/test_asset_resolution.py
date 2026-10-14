"""Offline tests for rospkg-style asset resolution (no cwd dependence).

The upstream planner defaults are current-directory relative
(module2_planner.py: "config/arm_kinematics.yaml",
"models_hybrid_1m/g1_left_hybrid_best.pt"), and the directory it names does
not exist in this repository. resolve_assets() takes an explicit root instead,
so these tests point it at a temporary tree and at the real package.
"""
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from arm_planning import plan_conversion as pc  # noqa: E402

PKG = Path(os.path.dirname(os.path.abspath(__file__))).parent


def _populate(root, checkpoint_dir="models", meshes=True, config=True, masks=True):
    """Build a minimal fake package tree; file contents are irrelevant here."""
    root = Path(root)
    if config:
        (root / "config").mkdir(parents=True, exist_ok=True)
        (root / "config" / "arm_kinematics.yaml").write_text("robot_name: g1_23dof\n")
    if checkpoint_dir:
        (root / checkpoint_dir).mkdir(parents=True, exist_ok=True)
        for arm in pc.ARMS:
            (root / checkpoint_dir / ("g1_%s_hybrid_best.pt" % arm)).write_text("x")
    if masks:
        (root / "collision_masks").mkdir(parents=True, exist_ok=True)
        for arm in pc.ARMS:
            (root / "collision_masks" / ("%s_static_collision_mask.npy" % arm)).write_text("x")
    if meshes:
        (root / "meshes").mkdir(parents=True, exist_ok=True)
        for name in pc.required_mesh_files():
            (root / "meshes" / name).write_text("x")
    return root


class ResolveComplete(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)

    def test_complete_tree_resolves_ready(self):
        _populate(self.tmp)
        assets = pc.resolve_assets(self.tmp)
        self.assertTrue(assets.ready, assets.blocker_message())
        self.assertEqual(assets.missing, [])
        self.assertEqual(assets.blocker_message(), "")

    def test_every_path_is_absolute_under_the_given_root(self):
        _populate(self.tmp)
        assets = pc.resolve_assets(self.tmp)
        root = Path(self.tmp).resolve()
        paths = [assets.config_path, *assets.checkpoints.values(), *assets.masks.values()]
        for p in paths:
            self.assertTrue(
                str(Path(p).resolve()).startswith(str(root)),
                "%s escaped the project root" % p,
            )

    def test_resolution_ignores_the_current_working_directory(self):
        """Running from a different cwd must not change the resolved paths."""
        _populate(self.tmp)
        here = os.getcwd()
        before = pc.resolve_assets(self.tmp)
        other = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, other)
        try:
            os.chdir(other)
            after = pc.resolve_assets(self.tmp)
        finally:
            os.chdir(here)
        self.assertEqual(str(before.config_path), str(after.config_path))
        self.assertEqual(after.missing, [])
        self.assertTrue(after.ready)


class CheckpointDirectoryChoice(unittest.TestCase):
    """Doc 8.1 names models_hybrid_1m/; this repo ships models/. Accept both."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)

    def test_models_directory_is_reported(self):
        _populate(self.tmp, checkpoint_dir="models")
        assets = pc.resolve_assets(self.tmp)
        self.assertTrue(assets.ready)
        self.assertEqual(assets.checkpoint_dir_used, "models")

    def test_specification_directory_also_works(self):
        _populate(self.tmp, checkpoint_dir="models_hybrid_1m")
        assets = pc.resolve_assets(self.tmp)
        self.assertTrue(assets.ready)
        self.assertEqual(assets.checkpoint_dir_used, "models_hybrid_1m")

    def test_half_populated_directory_does_not_shadow_a_complete_one(self):
        _populate(self.tmp, checkpoint_dir="models_hybrid_1m")
        partial = Path(self.tmp) / "models"
        partial.mkdir()
        (partial / "g1_left_hybrid_best.pt").write_text("x")   # right arm absent
        assets = pc.resolve_assets(self.tmp)
        self.assertTrue(assets.ready, assets.blocker_message())
        self.assertEqual(assets.checkpoint_dir_used, "models_hybrid_1m")

    def test_missing_checkpoints_are_a_blocker_naming_both_directories(self):
        _populate(self.tmp, checkpoint_dir=None)
        assets = pc.resolve_assets(self.tmp)
        self.assertFalse(assets.ready)
        blocker = assets.blocker_message()
        self.assertIn("models", blocker)
        self.assertIn("models_hybrid_1m", blocker)


class MissingAssetsFailSafely(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)

    def test_resolution_never_raises_for_an_empty_tree(self):
        assets = pc.resolve_assets(self.tmp)   # must not raise
        self.assertFalse(assets.ready)
        self.assertGreaterEqual(len(assets.missing), 4)

    def test_missing_config_is_named(self):
        _populate(self.tmp, config=False)
        assets = pc.resolve_assets(self.tmp)
        self.assertFalse(assets.ready)
        self.assertIn("arm_kinematics.yaml", assets.blocker_message())

    def test_missing_mask_is_named_per_arm(self):
        _populate(self.tmp)
        os.remove(Path(self.tmp) / "collision_masks" / "right_static_collision_mask.npy")
        assets = pc.resolve_assets(self.tmp)
        self.assertFalse(assets.ready)
        self.assertIn("right_static_collision_mask.npy", assets.blocker_message())

    def test_missing_mesh_directory_is_named(self):
        _populate(self.tmp, meshes=False)
        assets = pc.resolve_assets(self.tmp)
        self.assertFalse(assets.ready)
        self.assertIn("meshes/", assets.blocker_message())

    def test_missing_single_mesh_is_named_with_the_download_hint(self):
        _populate(self.tmp)
        os.remove(Path(self.tmp) / "meshes" / "left_wrist_roll_rubber_hand.STL")
        assets = pc.resolve_assets(self.tmp)
        self.assertFalse(assets.ready)
        blocker = assets.blocker_message()
        self.assertIn("left_wrist_roll_rubber_hand.STL", blocker)
        self.assertIn("meshes/README.md", blocker)


class CollisionProbeMeshContract(unittest.TestCase):
    """required_mesh_files() must not drift from what the probe actually loads."""

    def test_required_meshes_match_collision_probe_source(self):
        probe = PKG / "scripts" / "g1_collision_probe.py"
        referenced = pc.scan_collision_probe_meshes(probe.read_text())
        self.assertEqual(
            sorted(referenced),
            sorted(pc.required_mesh_files()),
            "g1_collision_probe.py mesh references changed; update "
            "STATIC_BODY_MESHES / PER_ARM_MESH_TEMPLATES",
        )

    def test_mesh_list_covers_both_arms(self):
        names = pc.required_mesh_files()
        for arm in pc.ARMS:
            self.assertIn("%s_shoulder_yaw_link.STL" % arm, names)
            self.assertIn("%s_elbow_link.STL" % arm, names)
            self.assertIn("%s_wrist_roll_rubber_hand.STL" % arm, names)

    def test_mesh_list_has_no_duplicates(self):
        names = pc.required_mesh_files()
        self.assertEqual(len(names), len(set(names)))


class RealPackageTree(unittest.TestCase):
    """Resolution against the checked-in package, documenting its real state."""

    def test_config_masks_and_checkpoints_are_present(self):
        assets = pc.resolve_assets(PKG)
        self.assertTrue(assets.config_path.is_file())
        self.assertEqual(assets.checkpoint_dir_used, "models")
        for arm in pc.ARMS:
            self.assertTrue(assets.checkpoints[arm].is_file())
            self.assertTrue(assets.masks[arm].is_file())

    def test_hand_collision_meshes_are_the_only_outstanding_blocker(self):
        """KNOWN BLOCKER, reported rather than worked around.

        meshes/ holds a partial download: *_wrist_roll_rubber_hand.STL (needed
        by g1_collision_probe.py) were never fetched. *_hand_palm_link.STL is a
        DIFFERENT geometry and substituting it would silently change the
        collision model, so the node reports NOT_READY instead.
        """
        assets = pc.resolve_assets(PKG)
        self.assertFalse(assets.ready, "meshes appear complete; update this test")
        self.assertEqual(len(assets.missing), 1, assets.missing)
        blocker = assets.missing[0]
        self.assertIn("left_wrist_roll_rubber_hand.STL", blocker)
        self.assertIn("right_wrist_roll_rubber_hand.STL", blocker)
        meshes = assets.meshes_dir
        self.assertTrue((meshes / "left_hand_palm_link.STL").is_file())
        self.assertFalse((meshes / "left_wrist_roll_rubber_hand.STL").is_file())


if __name__ == "__main__":
    unittest.main()
