"""fetch_deliver at the node level: waypoint blockers, label, dry-run traversal.

rospy is stubbed exactly as in test_node_safety.py (whose stub is reused), and
g1_slam is imported from the sibling package so the REAL standing-waypoint
resolver runs against a temporary map directory. No ROS master.
"""
import os
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "slam", "src"))   # g1_slam, read-only

import test_node_safety as tns  # noqa: E402  (installs the rospy stub)
from orchestrator import routes as r  # noqa: E402
from orchestrator import standing_waypoints as sw  # noqa: E402
from g1_slam import standing_store as store  # noqa: E402

on = tns.on
ROSPY = tns.ROSPY

IDENT = store.identity("lab_map_test", "20261017_031723", "map", "pelvis")


def make_map(tmp, name="lab_map_test", waypoints=("table_01", "chair_01"), manifest=True,
             manifest_name=None):
    map_dir = os.path.join(tmp, name)
    os.makedirs(map_dir)
    if manifest:
        with open(os.path.join(map_dir, "manifest.yaml"), "w", encoding="utf-8") as fh:
            fh.write("map_name: %s\ncreated_at: '20261017_031723'\n"
                     % (manifest_name or "lab_map_test"))
    path = os.path.join(map_dir, "standing_waypoints.yaml")
    for i, wp in enumerate(waypoints):
        store.add(path, IDENT, x=float(i), y=-float(i), yaw=0.5 * i,
                  recorded_at="2026-10-17T03:21:54", stamp=1000.0 + i, name=wp)
    return map_dir


class ResolverAgainstARealFile(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_both_waypoints_resolve_with_their_frames(self):
        make_map(self.tmp.name)
        resolved, blockers = sw.resolve_standing_waypoints(
            self.tmp.name, "lab_map_test", {"pickup": "table_01", "delivery": "chair_01"})
        self.assertEqual(blockers, [])
        self.assertEqual(sorted(resolved), ["delivery", "pickup"])
        key, record, frame = resolved["pickup"]
        self.assertEqual(key, "table_01")
        self.assertEqual(frame, "map")
        self.assertEqual(record["x"], 0.0)
        self.assertEqual(resolved["delivery"][1]["yaw"], 0.5)

    def test_a_missing_record_is_a_blocker_naming_the_leg(self):
        make_map(self.tmp.name, waypoints=("table_01",))
        resolved, blockers = sw.resolve_standing_waypoints(
            self.tmp.name, "lab_map_test", {"pickup": "table_01", "delivery": "chair_01"})
        self.assertEqual(sorted(resolved), ["pickup"])
        self.assertEqual(len(blockers), 1)
        self.assertIn("delivery", blockers[0])
        self.assertIn("chair_01", blockers[0])
        self.assertIn("standing_waypoints (slam)", blockers[0])

    def test_an_empty_name_is_a_blocker_not_a_default(self):
        make_map(self.tmp.name)
        _resolved, blockers = sw.resolve_standing_waypoints(
            self.tmp.name, "lab_map_test", {"pickup": "table_01", "delivery": ""})
        self.assertEqual(len(blockers), 1)
        self.assertIn("names no standing waypoint", blockers[0])

    def test_a_map_mismatch_is_refused(self):
        """Coordinates recorded in one map must never be driven to in another."""
        make_map(self.tmp.name, manifest_name="another_map")
        resolved, blockers = sw.resolve_standing_waypoints(
            self.tmp.name, "lab_map_test", {"pickup": "table_01", "delivery": "chair_01"})
        self.assertEqual(resolved, {})
        self.assertEqual(len(blockers), 2)
        self.assertIn("another_map", blockers[0])

    def test_a_missing_file_is_a_blocker(self):
        os.makedirs(os.path.join(self.tmp.name, "empty_map"))
        _resolved, blockers = sw.resolve_standing_waypoints(
            self.tmp.name, "empty_map", {"pickup": "table_01"})
        self.assertEqual(len(blockers), 1)
        self.assertIn("record_waypoints.py", blockers[0])


class NodeLevelFetchDeliver(tns.PreflightPatch):
    def setUp(self):
        super(NodeLevelFetchDeliver, self).setUp()
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        make_map(self.tmp.name)

    def _node(self, enable_motion=False, blockers=None, side="left", handover_arm="left"):
        n = tns.node(enable_motion=enable_motion, blockers=blockers, side=side)
        n.maps_dir = self.tmp.name
        n.map_name = "lab_map_test"
        ROSPY._params["/handover/arm"] = handover_arm
        return n

    def _req(self, target="table_01", destination="chair_01", label="", dry_run=False):
        return tns.request(target=target, destination=destination, label=label,
                           dry_run=dry_run)

    def test_a_dry_run_traverses_every_step_in_order(self):
        response = self._node().run("fetch_deliver", self._req())
        self.assertTrue(response.success, response.message)
        self.assertTrue(response.dry_run)
        self.assertFalse(response.motion_issued)
        self.assertEqual(list(response.steps_completed), r.FETCH_DELIVER.step_names)
        self.assertEqual(list(response.blockers), [])

    def test_a_missing_destination_is_refused_before_the_lock(self):
        n = self._node()
        response = n.run("fetch_deliver", self._req(destination=""))
        self.assertFalse(response.success)
        self.assertIn("destination", response.message)
        self.assertFalse(n.lock.busy)

    def test_an_unknown_waypoint_is_reported_as_a_blocker(self):
        response = self._node().run("fetch_deliver", self._req(destination="sofa_01"))
        self.assertTrue(response.success)          # the rehearsal itself still runs
        self.assertEqual(len(response.blockers), 1)
        self.assertIn("sofa_01", response.blockers[0])
        self.assertIn("a real run would be refused", response.message)

    def test_a_real_run_with_an_unknown_waypoint_moves_nothing(self):
        response = self._node(enable_motion=True).run(
            "fetch_deliver", self._req(destination="sofa_01"))
        self.assertFalse(response.success)
        self.assertFalse(response.motion_issued)
        self.assertEqual(list(response.steps_completed), [])
        self.assertIn("NOT_READY", response.message)

    def test_handover_node_for_the_other_arm_is_a_blocker(self):
        response = self._node(side="left", handover_arm="right").run(
            "fetch_deliver", self._req())
        self.assertEqual(len(response.blockers), 1)
        self.assertIn("handover_node", response.blockers[0])
        self.assertIn("right arm", response.blockers[0])

    def test_the_default_label_fills_an_empty_request(self):
        n = self._node()
        self.assertEqual(n._label(self._req(label="")), "bottle")
        self.assertEqual(n._label(self._req(label="  cup ")), "cup")

    def test_a_real_run_with_a_clean_preflight_reaches_the_backend(self):
        """The arm_grasp backend is the one constructed, and only then."""
        n = self._node(enable_motion=True)
        built = []

        class FakeBackend(object):
            def __init__(self, side, timeouts):
                built.append(side)
                raise RuntimeError("stop here: construction is what this test checks")

        original = on.ArmGraspBackend
        on.ArmGraspBackend = FakeBackend
        try:
            response = n.run("fetch_deliver", self._req())
        finally:
            on.ArmGraspBackend = original
        self.assertEqual(built, ["left"])
        self.assertFalse(response.success)
        self.assertIn("cannot build the execution backend", response.message)

    def test_only_fetch_deliver_selects_the_arm_grasp_backend(self):
        """Backend choice is by route; construction itself needs ROS and is not run."""
        self.assertNotIn("deliver_task", r.ROUTES_USING_STANDING_WAYPOINTS)
        self.assertNotIn("grasp", r.ROUTES_USING_STANDING_WAYPOINTS)
        self.assertIn("fetch_deliver", r.ROUTES_USING_STANDING_WAYPOINTS)


if __name__ == "__main__":
    unittest.main()
