"""slam_manager's save / startup-check paths with ROS stubbed out (no ROS, no robot).

The real pcd_to_map is replaced by a grid writer; everything else (merge, floor estimate, staging, verification,
service reply, status, `current` link) is the real slam_manager code.
"""
import importlib.util
import math
import os
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, HERE)

from test_cloud_utils import make_room  # noqa: E402
from test_map_store import write_room_grid  # noqa: E402

SCRIPT = os.path.join(HERE, "..", "scripts", "slam_manager.py")


def _stubs():
    rospy = types.ModuleType("rospy")
    for n in ("loginfo", "logwarn", "logerr", "logerr_throttle", "logfatal"):
        setattr(rospy, n, lambda *a, **k: None)
    mods = {"rospy": rospy, "rospkg": types.ModuleType("rospkg")}
    sm = types.ModuleType("sensor_msgs"); sm.msg = types.ModuleType("sensor_msgs.msg"); sm.msg.PointCloud2 = object
    st = types.ModuleType("std_msgs"); st.msg = types.ModuleType("std_msgs.msg")
    st.msg.Bool = st.msg.Empty = st.msg.String = object
    ss = types.ModuleType("std_srvs"); ss.srv = types.ModuleType("std_srvs.srv")

    class Resp:
        def __init__(self, success, message):
            self.success, self.message = success, message
    ss.srv.Trigger, ss.srv.TriggerResponse = object, Resp
    mods.update({"sensor_msgs": sm, "sensor_msgs.msg": sm.msg, "std_msgs": st, "std_msgs.msg": st.msg,
                 "std_srvs": ss, "std_srvs.srv": ss.srv})
    return mods


class TestSlamManagerPaths(unittest.TestCase):
    def setUp(self):
        self.patch = mock.patch.dict(sys.modules, _stubs())
        self.patch.start()
        spec = importlib.util.spec_from_file_location("slam_manager_under_test", SCRIPT)
        self.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.mod)
        self.tmp = tempfile.TemporaryDirectory()
        m = object.__new__(self.mod.SlamManager)
        m.mode, m.maps_dir, m.map_name = "mapping", self.tmp.name, "lab"
        m.lock, m.pending, m.scans = threading.Lock(), [], 0
        m.cloud = make_room().astype(np.float32)
        m.voxel, m.min_points, m.max_points, m.full_warned = 0.05, 5000, 8000000, False
        m.floor_z, m.obs_min_h, m.obs_max_h, m.resolution = None, 0.15, 1.8, 0.05
        m.map_to_saved_rpy = [math.pi, 0.0, 0.0]
        m.status = []
        m.set_status = m.status.append
        m._grid_runner = lambda pcd, out, floor_up: write_room_grid(out)
        self.m = m

    def tearDown(self):
        self.patch.stop()
        self.tmp.cleanup()

    def test_save_service_succeeds_and_reports_next_steps(self):
        r = self.m.save_cb(None)
        self.assertTrue(r.success, r.message)
        self.assertIn("check_saved_map.py lab", r.message)
        self.assertEqual(os.readlink(os.path.join(self.tmp.name, "current")), "lab")
        self.assertTrue(os.path.isfile(os.path.join(self.tmp.name, "lab", "map.pcd")))

    def test_save_service_reports_failure_honestly_and_leaves_nothing(self):
        def boom(pcd, out, floor_up):
            raise RuntimeError("pcd_to_map failed (exit 1): no ground")
        self.m._grid_runner = boom
        r = self.m.save_cb(None)
        self.assertFalse(r.success)
        self.assertIn("save failed", r.message)
        self.assertEqual(os.listdir(self.tmp.name), [])
        self.assertTrue(any("FAILED" in s for s in self.m.status))

    def test_too_few_points_is_a_failure(self):
        self.m.cloud = self.m.cloud[:100]
        r = self.m.save_cb(None)
        self.assertFalse(r.success)
        self.assertIn("Drive the robot around", r.message)

    def test_save_only_in_mapping_mode(self):
        self.m.mode = "localization"
        self.assertFalse(self.m.save_cb(None).success)

    def test_localization_start_status_verifies_the_map(self):
        self.assertTrue(self.m.save_cb(None).success)
        self.m.mode, self.m.map_name = "localization", "lab"
        self.assertIn("verified", self.m._localization_start_status())
        self.m.map_name = "nope"
        self.assertIn("unusable", self.m._localization_start_status())


if __name__ == "__main__":
    unittest.main()
