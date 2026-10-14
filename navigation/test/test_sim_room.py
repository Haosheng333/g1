"""Offline: the shipped sim room + semantic goals + real map/tier/planner/DWA code (no ROS).

    cd navigation/test && python3 -m unittest test_sim_room -v
"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "slam", "src"))
sys.path.insert(0, HERE)

from costmap import Costmap  # noqa: E402
from global_planner import AStarPlanner  # noqa: E402
from semantic_goal import find_approach_pose, load_semantic, lookup_object  # noqa: E402
from g1_slam import grid_utils as gu  # noqa: E402
from test_costmap_planner_dwa import Sim, wrap  # noqa: E402

SIM = os.path.join(HERE, "..", "sim", "sim_room")


class SimRoomTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        grid, res, ox, oy = gu.load_map_yaml(os.path.join(SIM, "map.yaml"))
        cls.tiers = gu.compose_tiers(grid, res)
        cls.cm = Costmap.from_grid(cls.tiers, res, ox, oy)
        cls.sem = load_semantic(os.path.join(SIM, "semantic.yaml"))

    def test_map_loads_in_z_up_map_frame(self):
        self.assertEqual(self.tiers.shape, (180, 220))
        cm = self.cm
        self.assertTrue(cm.is_free(*cm.world_to_map(1.0, 1.0)))
        self.assertFalse(cm.is_free(*cm.world_to_map(2.5, 2.0)))      # table
        self.assertFalse(cm.is_free(*cm.world_to_map(7.4, 5.4)))      # sofa
        self.assertFalse(cm.is_free(*cm.world_to_map(-0.5, 4.0)))     # unknown outside the walls
        self.assertTrue(cm.is_free(*cm.world_to_map(5.0, 4.0)))       # door centre

    def test_semantic_goals_resolve_and_are_reached(self):
        for name, approach in (("table", 0.6), ("sofa", 0.8)):
            obj = lookup_object(self.sem, name)
            pl = AStarPlanner()
            tgt = find_approach_pose(self.cm, pl, (obj["x"], obj["y"]), approach, (1.0, 1.0))
            self.assertIsNotNone(tgt, name)
            x, y, yaw = tgt
            d = math.hypot(x - obj["x"], y - obj["y"])
            self.assertTrue(approach - 0.15 < d < approach + 0.6, "%s approach distance %.2f" % (name, d))
            self.assertLess(abs(wrap(yaw - math.atan2(obj["y"] - y, obj["x"] - x))), 1e-6)   # faces the object
            sim = Sim(self.cm, (1.0, 1.0, 0.0))
            status, t = sim.drive((x, y, yaw), planner=pl, max_time=150.0)
            self.assertEqual(status, "SUCCESS", "%s: %s after %.0fs" % (name, status, t))
            self.assertLess(math.hypot(sim.pose[0] - x, sim.pose[1] - y), 0.10)
            self.assertLess(abs(wrap(sim.pose[2] - yaw)), 0.1)
            self.assertGreater(sim.min_clear, 0.0, "%s: footprint touched an obstacle" % name)

    def test_start_in_other_room_goes_through_the_door(self):
        pl = AStarPlanner()
        path = pl.plan(self.cm, (1.0, 6.5), (9.0, 1.0))
        self.assertTrue(any(abs(x - 5.0) < 0.2 and 3.5 < y < 4.5 for x, y in path))


if __name__ == "__main__":
    unittest.main()
