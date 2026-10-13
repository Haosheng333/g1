"""SLAM -> navigation contract, end to end and offline (no ROS).

    cd navigation/test && python3 -m unittest test_slam_nav_contract -v

Runs the REAL slam code (g1_slam.grid_utils: pgm/yaml loading + tier composing, exactly what
grid_composer.py does) and the REAL semantic.yaml template written by slam_manager.py, feeds the
resulting /map and semantic file to the real navigation_node state machine and drives a simulated
robot to a named object.  The map has a NEGATIVE origin and is saved the way pcd_to_map writes it
(P5 pgm, row 0 = top, 254 free / 0 occupied / 205 unknown, negate 0), so axis flips, origin offsets
and tier values are all exercised.
"""
import math
import os
import re
import sys
import tempfile
import unittest

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "slam", "src"))

import test_navigation_node as tn  # noqa: E402  (installs the rospy stubs, imports navigation_node)
from g1_slam import grid_utils as gu  # noqa: E402
import costmap as cmod  # noqa: E402

nn, WORLD, _Obj, _Vec, OccupancyGrid = tn.nn, tn.WORLD, tn._Obj, tn._Vec, tn.OccupancyGrid

SLAM_MANAGER = os.path.join(HERE, "..", "..", "slam", "scripts", "slam_manager.py")
RES, ORIGIN = 0.05, (-4.0, -3.0)          # 8 x 6 m room, origin like a real map (not 0,0)


def _write(path, text):
    with open(path, "w") as f:
        f.write(text)


def slam_semantic_template():
    """The semantic.yaml that save_map() really writes into a new map folder."""
    from g1_slam import semantic_store as ss
    return ss.empty_file_text()


def write_saved_map(folder):
    """Room with a table, written like pcd_to_map: map.pgm + map.yaml."""
    w, h = int(8.0 / RES), int(6.0 / RES)
    grid = np.zeros((h, w), np.int8)                           # row 0 = min y (OccupancyGrid order)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    # table 1.0 x 0.6 m centred at map (0.5, 0.0)
    x0, x1 = int((0.0 - ORIGIN[0]) / RES), int((1.0 - ORIGIN[0]) / RES)
    y0, y1 = int((-0.3 - ORIGIN[1]) / RES), int((0.3 - ORIGIN[1]) / RES)
    grid[y0:y1, x0:x1] = 100
    img = np.where(grid == 100, 0, 254).astype(np.uint8)[::-1, :]     # image row 0 = top = max y
    with open(os.path.join(folder, "map.pgm"), "wb") as f:
        f.write(b"P5\n# CREATOR: pcd_to_map\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())
    with open(os.path.join(folder, "map.yaml"), "w") as f:
        yaml.safe_dump({"image": "map.pgm", "resolution": RES, "origin": [ORIGIN[0], ORIGIN[1], 0.0],
                        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}, f)
    return grid


def as_occupancy_grid(tiers):
    h, w = tiers.shape
    g = OccupancyGrid()
    g.info = _Obj(width=w, height=h, resolution=RES, origin=_Obj(position=_Obj(x=ORIGIN[0], y=ORIGIN[1], z=0.0)))
    g.data = tuple(int(v) for v in tiers.flatten())
    return g


class TestSlamNavContract(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.raw = write_saved_map(self.dir.name)
        grid, res, ox, oy = gu.load_map_yaml(os.path.join(self.dir.name, "map.yaml"))
        self.assertEqual((res, ox, oy), (RES, ORIGIN[0], ORIGIN[1]))
        # grid_composer defaults: danger 0.25, risk 0.50, hole fill 3
        self.tiers = gu.compose_tiers(grid, res, danger_m=0.25, risk_m=0.50, hole_fill_iterations=3)
        # the file ships with `waypoints: {}`; surveying (tools/survey_semantic.py) fills it through semantic_store
        from g1_slam import semantic_store as ss
        self.sem_path = os.path.join(self.dir.name, "semantic.yaml")
        _write(self.sem_path, slam_semantic_template())
        ss.add(self.sem_path, "table", 2.5, 1.2, 1.57)
        ss.add(self.sem_path, "sofa", -1.0, 3.4, 0.0, approach_yaw=3.14)

    def tearDown(self):
        self.dir.cleanup()

    def test_saved_map_matches_ground_truth(self):
        self.assertEqual(self.tiers.shape, self.raw.shape)
        self.assertTrue(np.all((self.tiers == 100) == (self.raw == 100)))     # no flip, no shift
        self.assertEqual(set(np.unique(self.tiers)) - {0, 33, 66, 100}, set())

    def test_tier_codes_agree_between_slam_and_navigation(self):
        self.assertEqual((gu.UNKNOWN, gu.WALKABLE, gu.RISK, gu.DANGER, gu.OBSTACLE), (-1, 0, 33, 66, 100))
        for name in ("UNKNOWN", "WALKABLE", "RISK", "DANGER", "OBSTACLE"):
            if hasattr(cmod, name):
                self.assertEqual(getattr(cmod, name), getattr(gu, name), name)

    def test_template_as_shipped_loads_empty_and_filled_loads_objects(self):
        import semantic_goal as sg
        empty = os.path.join(self.dir.name, "empty.yaml")
        _write(empty, slam_semantic_template())
        self.assertEqual(sg.load_semantic(empty), {})
        sem = sg.load_semantic(self.sem_path)
        self.assertIn("table", sem)
        self.assertAlmostEqual(sem["table"]["x"], 2.5)             # template example values
        self.assertIsNotNone(sem["sofa"]["approach_yaw"])

    def test_navigate_to_named_object_in_a_real_saved_map(self):
        WORLD.now, WORLD.tf_ok = 0.0, True
        WORLD.pose = (-3.0, -2.0, 0.3)
        WORLD.published.clear()
        # put the object where the table actually is in THIS map
        _write(self.sem_path, "waypoints:\n  table: {x: 0.5, y: 0.0, yaw: 0.0}\n")
        WORLD.params = {"~semantic_file": self.sem_path, "~use_localization_gate": True}
        n = nn.NavigationNode()
        n.map_cb(as_occupancy_grid(self.tiers))
        n.state_cb(nn.Bool(data=True))
        resp = WORLD.svc(_Obj(class_name="Table", approach_distance=0.8))
        self.assertTrue(resp.success, resp.message)

        case = tn.NavTest("test_rviz_goal_success_with_yaw")      # reuse its closed-loop stepper
        case.DT = 0.1
        case.step(n, 120)
        sts = case.statuses()
        self.assertEqual(sts[-1], "SUCCESS", sts)
        x, y, yaw = WORLD.pose
        self.assertTrue(0.6 < math.hypot(x - 0.5, y - 0.0) < 1.4, (x, y))
        facing = math.atan2(0.0 - y, 0.5 - x)
        self.assertLess(abs(math.atan2(math.sin(yaw - facing), math.cos(yaw - facing))), 0.1)
        # ground truth: the real footprint never touched the saved map's obstacles
        self.assertGreater(getattr(case, "min_clear", 9.0), 0.0)


if __name__ == "__main__":
    unittest.main()
