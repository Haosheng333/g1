"""Section F conformance (items 22-26: mapping and map manager), run without ROS or a robot.

    cd navigation/test && python3 -m unittest test_section_f -v

22  pipeline to a TIERED grid      FAST-LIO2 -> slam_manager -> pcd_to_map -> grid_composer (tiers); no RTAB-Map
23  saving is reliable             staging dir, verification, honest failure, never a half-written map  (slam/test/test_map_store.py)
24  the manager owns the lifecycle mapping / localization sessions, the /slam/manager/state flag, map checked at start
25  semantic.yaml is MEASURED      survey tool + check_saved_map.py proves every object has a reachable approach pose
26  the grid comes from the pipeline  no hand-made real map in the repo; only the labelled simulator map
Alignment with G: a mapping session never starts navigation or locomotion nodes, so nothing can move the robot.
"""
import glob
import importlib.util
import os
import re
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "slam", "src"))
sys.path.insert(0, os.path.join(REPO, "navigation", "scripts"))

from g1_slam import grid_utils as gu  # noqa: E402
from g1_slam import semantic_store as ss  # noqa: E402

SIM = os.path.join(REPO, "navigation", "sim", "sim_room")


def _load_tool(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(REPO, "tools", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _read(*p):
    with open(os.path.join(REPO, *p), encoding="utf-8", errors="replace") as fh:
        return fh.read()


def _launch(rel):
    return ET.fromstring(re.sub(r"<!--.*?-->", "", _read(*rel), flags=re.S))


def _edit_pgm(folder, fn):
    """Apply fn(img, res, ox, oy) to the map image (image row 0 = top) and write it back."""
    grid, res, ox, oy = gu.load_map_yaml(os.path.join(folder, "map.yaml"))
    img = gu.read_pgm(os.path.join(folder, "map.pgm")).copy()
    fn(img, res, ox, oy)
    h, w = img.shape
    with open(os.path.join(folder, "map.pgm"), "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())


def _world_to_px(img, res, ox, oy, x, y):
    return img.shape[0] - 1 - int((y - oy) / res), int((x - ox) / res)    # row, col


class TestPipeline22(unittest.TestCase):
    def test_no_rtabmap_anywhere_in_slam(self):
        for lf in glob.glob(os.path.join(REPO, "slam", "launch", "*.launch")):
            with open(lf) as fh:
                text = re.sub(r"<!--.*?-->", "", fh.read(), flags=re.S)
            self.assertNotRegex(text, r'pkg="rtabmap', lf)

    def test_mapping_session_runs_fastlio_and_the_manager(self):
        root = _launch(("slam", "launch", "mapping.launch"))
        incs = [i.get("file") for i in root.iter("include")]
        self.assertTrue(any("slam_bringup.launch" in i for i in incs), incs)
        mgr = [n for n in root.iter("node") if n.get("type") == "slam_manager.py"]
        self.assertEqual(len(mgr), 1)
        self.assertIn(("mode", "mapping"), [(p.get("name"), p.get("value")) for p in mgr[0].iter("param")])
        self.assertIn("fastlio_mapping.launch", _read("slam", "launch", "slam_bringup.launch"))

    def test_the_tiered_grid_comes_from_grid_composer_on_the_saved_map(self):
        root = _launch(("slam", "launch", "localization.launch"))
        gc = [n for n in root.iter("node") if n.get("type") == "grid_composer.py"]
        self.assertEqual(len(gc), 1)
        params = {p.get("name"): p.get("value") for p in gc[0].iter("param")}
        self.assertTrue(params["map_yaml"].endswith("/map.yaml"))
        self.assertEqual(params["frame_id"], "map")
        self.assertIn("compose_tiers", _read("slam", "scripts", "grid_composer.py"))

    def test_only_one_publisher_of_the_tiered_map(self):
        """map_server.launch also publishes /map (plain, in camera_init): standalone viewing only."""
        for pkg in ("slam", "navigation", "system_bringup"):
            for lf in glob.glob(os.path.join(REPO, pkg, "launch", "*.launch")):
                if os.path.basename(lf) == "map_server.launch":
                    continue
                with open(lf) as fh:
                    text = re.sub(r"<!--.*?-->", "", fh.read(), flags=re.S)
                self.assertNotIn("map_server.launch", text, lf)
                self.assertNotRegex(text, r'pkg="map_server"', lf)

    def test_pcd_to_map_is_the_only_grid_source(self):
        src = _read("slam", "scripts", "slam_manager.py")
        self.assertIn('"pcd_to_map"', src)
        self.assertNotIn("backup", src.lower())


class TestSaving23(unittest.TestCase):
    def test_slam_manager_saves_through_the_verified_atomic_path(self):
        src = _read("slam", "scripts", "slam_manager.py")
        self.assertIn("ms.save_map(", src)
        self.assertRegex(src, r"TriggerResponse\(success=False,\s*message=\"save failed")      # failure is reported
        self.assertNotRegex(src, r"os\.makedirs\(out\)")                                       # no direct, non-atomic writes

    def test_manifest_comes_from_the_session_not_constants(self):
        src = _read("slam", "src", "g1_slam", "map_store.py")
        for key in ("num_points", "floor_z_camera_init", "resolution", "voxel"):
            self.assertRegex(src, r'"%s":\s*(int|float)\(' % key)


class TestLifecycle24(unittest.TestCase):
    def test_both_modes_exist_with_their_services(self):
        src = _read("slam", "scripts", "slam_manager.py")
        for needle in ('"mapping", "localization"', "/slam/manager/save_current_map", "/slam/manager/relocalize",
                       "/slam/manager/state", "/slam/manager/status"):
            self.assertIn(needle, src)

    def test_localization_checks_the_map_before_the_robot_depends_on_it(self):
        src = _read("slam", "scripts", "slam_manager.py")
        self.assertIn("_localization_start_status", src)
        self.assertIn("ms.verify_map_dir(", src)

    def test_mapping_never_starts_navigation_or_locomotion(self):
        text = _read("slam", "launch", "mapping.launch") + _read("slam", "launch", "slam_bringup.launch")
        for forbidden in ("navigation_node", "cmd_vel_gate", "locomotion_bridge", "g1_navigation", "enable_motion",
                          "system_bringup"):
            self.assertNotIn(forbidden, re.sub(r"<!--.*?-->", "", text, flags=re.S), forbidden)


class TestSemantic25(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = os.path.join(self.tmp.name, "room")
        shutil.copytree(SIM, self.dir)
        self.tool = _load_tool("check_saved_map")

    def tearDown(self):
        self.tmp.cleanup()

    def run_checks(self, **kw):
        kw.setdefault("grid_only", True)
        kw.setdefault("start", (1.0, 1.0))
        return self.tool.run_checks(self.dir, **kw)

    def failed(self, results):
        return [n for ok, n, _ in results if not ok]

    def test_simulator_map_passes_with_both_routes_objects(self):
        res = self.run_checks(require=("table", "sofa"))
        self.assertEqual(self.failed(res), [], res)
        self.assertGreaterEqual(len(res), 5)

    def test_unsurveyed_required_object_fails(self):
        res = self.run_checks(require=("table", "sofa", "bed"))
        self.assertEqual(self.failed(res), ["object 'bed' is surveyed"])

    def test_object_with_no_standable_space_around_it_fails(self):
        def fill(img, res, ox, oy):
            r, c = _world_to_px(img, res, ox, oy, 7.4, 5.4)
            k = int(2.2 / res)
            img[max(r - k, 0):r + k, max(c - k, 0):c + k] = 0           # solid block over the sofa
        _edit_pgm(self.dir, fill)
        self.assertIn("object 'sofa' has a reachable approach pose", self.failed(self.run_checks()))

    def test_object_outside_the_map_fails(self):
        ss.add(os.path.join(self.dir, "semantic.yaml"), "lamp", 50.0, 50.0)
        self.assertIn("object 'lamp' lies inside the map", self.failed(self.run_checks()))

    def test_a_map_split_in_two_fails(self):
        def wall(img, res, ox, oy):
            col = int((4.0 - ox) / res)
            img[:, col - 2:col + 3] = 0                                   # a solid wall across the whole room
        _edit_pgm(self.dir, wall)
        self.assertIn("standable space is one connected region", self.failed(self.run_checks()))

    def test_start_inside_a_wall_fails(self):
        res = self.run_checks(start=(-0.9, -0.9))
        self.assertTrue(any(n.startswith("mapping start") for n in self.failed(res)))

    def test_empty_map_fails(self):
        _edit_pgm(self.dir, lambda img, *a: img.fill(205))
        res = self.run_checks()
        self.assertEqual(self.failed(res)[0], "map folder is complete and consistent")

    def test_survey_tool_round_trip_without_ros(self):
        tool = _load_tool("survey_semantic")
        sem = os.path.join(self.dir, "semantic.yaml")
        self.assertTrue(tool.inside_map(self.dir, 2.5, 2.0))
        self.assertFalse(tool.inside_map(self.dir, 99.0, 2.0))
        self.assertEqual(tool.main(["remove", "table", "--map", self.dir]), 0)
        self.assertNotIn("table", ss.load(sem))
        ss.add(sem, "table", 2.5, 2.0)
        self.assertEqual(tool.main(["list", "--map", self.dir]), 0)


class TestNoHandMadeMaps26(unittest.TestCase):
    def test_only_the_labelled_simulator_map_is_committed(self):
        found = []
        for ext in ("pgm", "pcd"):
            found += glob.glob(os.path.join(REPO, "**", "*." + ext), recursive=True)
        found = [os.path.relpath(f, REPO) for f in found if "fast_lio" not in f and "g1_ws" not in f]   # g1_ws/src = symlinks
        self.assertEqual([f for f in found if not f.startswith(os.path.join("navigation", "sim"))], [])

    def test_generated_real_maps_stay_out_of_git_but_the_small_files_are_kept(self):
        ig = _read("slam", "maps", ".gitignore")
        for pat in ("*.pcd", "*.pgm", "*.yaml", "!*/manifest.yaml", "!*/semantic.yaml"):
            self.assertIn(pat, ig)

    def test_sim_map_is_reproducible_and_marked_synthetic(self):
        self.assertIn("Synthetic", _read("navigation", "sim", "sim_room", "semantic.yaml"))


if __name__ == "__main__":
    unittest.main()
