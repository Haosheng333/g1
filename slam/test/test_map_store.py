"""Map saving is atomic and verified; semantic.yaml is written safely (no ROS, no robot).

    cd slam && PYTHONPATH=src python3 -m unittest test_map_store -v   (or discover)
"""
import math
import os
import sys
import tempfile
import unittest

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, HERE)

from g1_slam import map_store as ms  # noqa: E402
from g1_slam import semantic_store as ss  # noqa: E402
from test_cloud_utils import make_room  # noqa: E402

RES = 0.05


def write_room_grid(out_dir, w=160, h=120, free=True, walls=True):
    """Stand-in for pcd_to_map: an 8 x 6 m room as map.pgm + map.yaml (image row 0 = top)."""
    img = np.full((h, w), 254 if free else 205, np.uint8)
    if walls:
        img[0, :] = img[-1, :] = img[:, 0] = img[:, -1] = 0
    with open(os.path.join(out_dir, "map.pgm"), "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())
    with open(os.path.join(out_dir, "map.yaml"), "w") as f:
        yaml.safe_dump({"image": "map.pgm", "resolution": RES, "origin": [-1.0, -1.0, 0.0], "negate": 0,
                        "occupied_thresh": 0.65, "free_thresh": 0.196}, f)


def runner_ok(pcd_zup, out_dir, floor_up):
    write_room_grid(out_dir)


def save(maps_dir, name="lab", runner=runner_ok):
    return ms.save_map(maps_dir, name, make_room(), 1.0, runner, 0.05, RES, [math.pi, 0.0, 0.0], (0.15, 1.8))


class TestSaveMap(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def test_success_writes_everything_and_points_current_at_it(self):
        final, info = save(self.dir)
        self.assertEqual(final, os.path.join(self.dir, "lab"))
        for f in ms.REQUIRED_FILES:
            self.assertGreater(os.path.getsize(os.path.join(final, f)), 0, f)
        self.assertEqual(os.readlink(os.path.join(self.dir, "current")), "lab")     # relative link
        self.assertEqual(ms.verify_map_dir(final)[0], [])
        self.assertEqual(sorted(os.listdir(self.dir)), ["current", "lab"])         # no staging dir left
        man = yaml.safe_load(open(os.path.join(final, "manifest.yaml")))
        self.assertEqual(man["num_points"], info["num_points"])
        self.assertEqual(man["mapping_start_xy_map"], [0.0, 0.0])
        self.assertEqual(ss.load(os.path.join(final, "semantic.yaml")), {})        # empty, to be surveyed

    def test_failed_grid_step_leaves_nothing_and_keeps_current(self):
        save(self.dir, "good")

        def boom(pcd, out, floor_up):
            raise ms.MapError("pcd_to_map failed (exit 1)")

        with self.assertRaises(ms.MapError):
            save(self.dir, "bad", boom)
        self.assertEqual(sorted(os.listdir(self.dir)), ["current", "good"])
        self.assertEqual(os.readlink(os.path.join(self.dir, "current")), "good")   # still the last GOOD map

    def test_grid_step_that_writes_nothing_is_an_error(self):
        with self.assertRaises(ms.MapError):
            save(self.dir, "x", lambda p, o, f: None)
        self.assertEqual(os.listdir(self.dir), [])

    def test_empty_grid_is_rejected_by_verification(self):
        with self.assertRaises(ms.MapError) as cm:
            save(self.dir, "x", lambda p, o, f: write_room_grid(o, free=False, walls=False))
        self.assertIn("almost empty", str(cm.exception))
        self.assertEqual(os.listdir(self.dir), [])

    def test_no_walls_is_rejected(self):
        with self.assertRaises(ms.MapError) as cm:
            save(self.dir, "x", lambda p, o, f: write_room_grid(o, walls=False))
        self.assertIn("no walls", str(cm.exception))

    def test_existing_name_is_never_overwritten(self):
        save(self.dir)
        with self.assertRaises(ms.MapError):
            save(self.dir)
        self.assertEqual(ms.unique_name(self.dir, "lab"), "lab_2")

    def test_bad_names(self):
        for bad in ("../x", "current", ".hidden"):
            with self.assertRaises(ms.MapError):
                ms.unique_name(self.dir, bad)


class TestVerify(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.final, _ = save(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_detects_missing_and_empty_files(self):
        os.remove(os.path.join(self.final, "map.pcd"))
        self.assertIn("missing map.pcd", ms.verify_map_dir(self.final)[0])
        open(os.path.join(self.final, "map.pcd"), "w").close()
        self.assertIn("map.pcd is empty", ms.verify_map_dir(self.final)[0])

    def test_detects_manifest_point_count_mismatch(self):
        p = os.path.join(self.final, "manifest.yaml")
        man = yaml.safe_load(open(p))
        man["num_points"] += 1
        yaml.safe_dump(man, open(p, "w"))
        self.assertTrue(any("manifest says" in x for x in ms.verify_map_dir(self.final)[0]))

    def test_detects_missing_manifest_keys(self):
        p = os.path.join(self.final, "manifest.yaml")
        man = yaml.safe_load(open(p))
        del man["floor_z_camera_init"]
        yaml.safe_dump(man, open(p, "w"))
        self.assertTrue(any("lacks floor_z_camera_init" in x for x in ms.verify_map_dir(self.final)[0]))

    def test_detects_corrupt_semantic(self):
        open(os.path.join(self.final, "semantic.yaml"), "w").write("waypoints: [1, 2]\n")
        self.assertTrue(any("semantic.yaml unusable" in x for x in ms.verify_map_dir(self.final)[0]))


class TestSemanticStore(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "semantic.yaml")
        open(self.path, "w").write(ss.empty_file_text())

    def tearDown(self):
        self.tmp.cleanup()

    def test_add_list_replace_remove(self):
        ss.add(self.path, "Table", 2.5, 1.2, 1.57)
        ss.add(self.path, "sofa", -1.0, 3.4, approach_yaw=3.14)
        got = ss.load(self.path)
        self.assertEqual(sorted(got), ["sofa", "table"])
        self.assertAlmostEqual(got["table"]["x"], 2.5)
        self.assertIsNone(got["table"]["approach_yaw"])
        self.assertAlmostEqual(got["sofa"]["approach_yaw"], 3.14)
        with self.assertRaises(ValueError):
            ss.add(self.path, "TABLE", 0, 0)                  # case-insensitive duplicate
        ss.add(self.path, "table", 3.0, 1.0, replace=True)
        self.assertAlmostEqual(ss.load(self.path)["table"]["x"], 3.0)
        ss.remove(self.path, "sofa")
        self.assertEqual(sorted(ss.load(self.path)), ["table"])
        with self.assertRaises(KeyError):
            ss.remove(self.path, "sofa")

    def test_rejects_bad_input(self):
        for name in ("", "a/b", "x" * 41, "Tab:le"):
            with self.assertRaises(ValueError):
                ss.add(self.path, name, 0, 0)
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                ss.add(self.path, "ok", bad, 0)
        self.assertEqual(ss.load(self.path), {})             # nothing half-written

    def test_file_keeps_its_instructions_and_has_no_temp_leftovers(self):
        ss.add(self.path, "table", 1, 2)
        self.assertTrue(open(self.path).read().startswith("# Named objects"))
        self.assertEqual(os.listdir(self.tmp.name), ["semantic.yaml"])

    def test_malformed_files_are_reported(self):
        for text in ("waypoints: [1]\n", "waypoints:\n  t: {x: 1}\n", "waypoints:\n  t: {x: a, y: 1}\n", "- 1\n"):
            open(self.path, "w").write(text)
            with self.assertRaises(ValueError):
                ss.load(self.path)

    def test_navigation_node_loader_reads_what_survey_writes(self):
        sys.path.insert(0, os.path.join(HERE, "..", "..", "navigation", "scripts"))
        import semantic_goal as sg
        ss.add(self.path, "Table", 2.5, 1.2, 1.57, approach_yaw=math.pi)
        got = sg.load_semantic(self.path)
        self.assertEqual(got["table"]["x"], 2.5)
        self.assertAlmostEqual(got["table"]["approach_yaw"], math.pi, 2)


if __name__ == "__main__":
    unittest.main()
