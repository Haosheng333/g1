"""record_waypoints and goto_waypoint with ROS stubbed out (no ROS, no robot).

Covers the refusals that keep a bad pose out of the file -- not localized, no
transform, stale transform, wrong frames, map mismatch -- and the goal mapping
that sends a saved pose back through navigation's EXACT-pose interface.
"""
import importlib.util
import math
import os
import sys
import tempfile
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from g1_slam import standing_store as store  # noqa: E402

RECORDER = os.path.join(HERE, "..", "scripts", "record_waypoints.py")
GOTO = os.path.join(HERE, "..", "scripts", "goto_waypoint.py")

IDENT = store.identity("g1_map_20261016_101500", "20261016_101500", "map", "pelvis")
OTHER = store.identity("g1_map_20261017_090000", "20261017_090000", "map", "pelvis")

NOW = 1000.0


# --------------------------------------------------------------------------- stubs
class FakeTime(object):
    def __init__(self, secs):
        self.secs = float(secs)

    def to_sec(self):
        return self.secs


class FakeDuration(FakeTime):
    pass


def _rospy_stub(clock):
    rospy = types.ModuleType("rospy")
    for name in ("loginfo", "logwarn", "logerr", "logfatal", "logdebug",
                 "logwarn_throttle", "logerr_throttle", "loginfo_throttle"):
        setattr(rospy, name, lambda *a, **k: None)
    rospy.Time = type("Time", (FakeTime,), {"now": staticmethod(lambda: FakeTime(clock["t"]))})
    rospy.Duration = FakeDuration
    rospy.Subscriber = lambda *a, **k: None
    rospy.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda *_a: None)
    rospy.Service = lambda *a, **k: None
    rospy.init_node = lambda *a, **k: None
    rospy.get_param = lambda name, default=None: default
    rospy.is_shutdown = lambda: False
    rospy.sleep = lambda *_a: None
    rospy.Rate = lambda *_a: types.SimpleNamespace(sleep=lambda: None)
    rospy.ROSInitException = type("ROSInitException", (Exception,), {})
    rospy.ROSInterruptException = type("ROSInterruptException", (Exception,), {})
    rospy.ROSException = type("ROSException", (Exception,), {})
    return rospy


class Vec(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class FakePose(object):
    def __init__(self):
        self.position, self.orientation = Vec(), Vec()


class FakePoseStamped(object):
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="", stamp=None)
        self.pose = FakePose()


def _module_stubs(clock):
    rospy = _rospy_stub(clock)
    mods = {"rospy": rospy}

    rospkg = types.ModuleType("rospkg")
    rospkg.RosPack = lambda: types.SimpleNamespace(get_path=lambda _p: "/nonexistent")
    mods["rospkg"] = rospkg

    tf2 = types.ModuleType("tf2_ros")
    tf2.Buffer = lambda *a, **k: None
    tf2.TransformListener = lambda *a, **k: None
    tf2.LookupException = type("LookupException", (Exception,), {})
    mods["tf2_ros"] = tf2

    std = types.ModuleType("std_msgs")
    std.msg = types.ModuleType("std_msgs.msg")
    std.msg.Bool = std.msg.String = object
    sens = types.ModuleType("sensor_msgs")
    sens.msg = types.ModuleType("sensor_msgs.msg")
    sens.msg.PointCloud2 = object
    geo = types.ModuleType("geometry_msgs")
    geo.msg = types.ModuleType("geometry_msgs.msg")
    geo.msg.PoseStamped = FakePoseStamped
    mods.update({"std_msgs": std, "std_msgs.msg": std.msg,
                 "sensor_msgs": sens, "sensor_msgs.msg": sens.msg,
                 "geometry_msgs": geo, "geometry_msgs.msg": geo.msg})
    return mods


def _load(path, name, clock):
    patch = mock.patch.dict(sys.modules, _module_stubs(clock))
    patch.start()
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, patch


def make_transform(parent="map", child="pelvis", x=1.25, y=-0.5, yaw=1.5707963,
                   stamp=NOW):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=parent, stamp=FakeTime(stamp)),
        child_frame_id=child,
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=x, y=y, z=0.0),
            rotation=types.SimpleNamespace(x=0.0, y=0.0,
                                           z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))))


class FakeBuffer(object):
    def __init__(self, transform=None, raise_with=None):
        self.transform, self.raise_with = transform, raise_with

    def lookup_transform(self, target, source, when, timeout=None):
        if self.raise_with is not None:
            raise self.raise_with
        return self.transform


# ----------------------------------------------------------------- the recorder
class RecorderPoseChecks(unittest.TestCase):
    def setUp(self):
        self.clock = {"t": NOW}
        self.mod, self.patch = _load(RECORDER, "record_waypoints_under_test", self.clock)
        self.addCleanup(self.patch.stop)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "standing_waypoints.yaml")

    def _recorder(self, localized=True, buffer_=None, **kw):
        r = object.__new__(self.mod.Recorder)
        r.map_frame, r.robot_frame = "map", "pelvis"
        r.max_tf_age, r.tf_timeout = 0.5, 1.0
        r.require_localized = True
        r.identity = IDENT
        r.waypoints_file = self.path
        r.prefix = "handover"
        r.snapshot = False
        r.snapshot_radius, r.snapshot_topic = 3.0, "/cloud_registered"
        r._cloud = r._cloud_at = None
        r._localized = localized
        r._tf_buffer = buffer_ if buffer_ is not None else FakeBuffer(make_transform())
        for k, v in kw.items():
            setattr(r, k, v)
        return r

    # --- the happy path -------------------------------------------------------
    def test_a_good_pose_is_read_from_the_map_transform(self):
        x, y, yaw, stamp = self._recorder().current_pose()
        self.assertAlmostEqual(x, 1.25, places=6)
        self.assertAlmostEqual(y, -0.5, places=6)
        self.assertAlmostEqual(yaw, 1.5707963, places=6)
        self.assertAlmostEqual(stamp, NOW, places=6)

    def test_saving_writes_the_record(self):
        self._recorder().save()
        _ident, waypoints = store.load(self.path)
        self.assertEqual(sorted(waypoints), ["handover_01"])
        self.assertAlmostEqual(waypoints["handover_01"]["yaw"], 1.5708, places=4)

    def test_saving_twice_gives_two_unique_names(self):
        r = self._recorder()
        r.save()
        r.save()
        _ident, waypoints = store.load(self.path)
        self.assertEqual(sorted(waypoints), ["handover_01", "handover_02"])

    def test_yaw_is_recovered_for_several_headings(self):
        for deg in (-179.0, -90.0, -30.0, 0.0, 45.0, 90.0, 179.0):
            want = math.radians(deg)
            r = self._recorder(buffer_=FakeBuffer(make_transform(yaw=want)))
            _x, _y, got, _s = r.current_pose()
            self.assertAlmostEqual(got, want, places=6, msg="%s deg" % deg)

    # --- refusals -------------------------------------------------------------
    def test_not_localized_is_refused(self):
        for state in (False, None):
            r = self._recorder(localized=state)
            with self.assertRaises(RuntimeError) as ctx:
                r.current_pose()
            self.assertIn("NOT localized", str(ctx.exception))
            self.assertIn("Odometry coordinates are NOT a substitute",
                          str(ctx.exception))

    def test_nothing_is_written_when_not_localized(self):
        self._recorder(localized=False).save()
        self.assertFalse(os.path.exists(self.path))

    def test_a_missing_transform_is_refused(self):
        r = self._recorder(buffer_=FakeBuffer(raise_with=RuntimeError("no such frame")))
        with self.assertRaises(RuntimeError) as ctx:
            r.current_pose()
        self.assertIn("no map -> pelvis transform", str(ctx.exception))

    def test_nothing_is_written_when_the_transform_is_missing(self):
        self._recorder(buffer_=FakeBuffer(raise_with=RuntimeError("boom"))).save()
        self.assertFalse(os.path.exists(self.path))

    def test_a_stale_transform_is_refused(self):
        r = self._recorder(buffer_=FakeBuffer(make_transform(stamp=NOW - 5.0)))
        with self.assertRaises(RuntimeError) as ctx:
            r.current_pose()
        self.assertIn("stale", str(ctx.exception))

    def test_a_transform_just_inside_the_age_limit_is_accepted(self):
        r = self._recorder(buffer_=FakeBuffer(make_transform(stamp=NOW - 0.4)))
        r.current_pose()

    def test_a_zero_stamp_is_refused(self):
        r = self._recorder(buffer_=FakeBuffer(make_transform(stamp=0.0)))
        with self.assertRaises(RuntimeError) as ctx:
            r.current_pose()
        self.assertIn("no usable stamp", str(ctx.exception))

    def test_an_odometry_frame_transform_is_refused(self):
        """The core guard: camera_init coordinates must never be recorded."""
        r = self._recorder(buffer_=FakeBuffer(make_transform(parent="camera_init")))
        with self.assertRaises(RuntimeError) as ctx:
            r.current_pose()
        self.assertIn("camera_init -> pelvis, expected map -> pelvis",
                      str(ctx.exception))

    def test_a_wrong_child_frame_is_refused(self):
        r = self._recorder(buffer_=FakeBuffer(make_transform(child="torso_link")))
        with self.assertRaises(RuntimeError):
            r.current_pose()

    def test_a_leading_slash_on_the_frames_is_tolerated(self):
        r = self._recorder(buffer_=FakeBuffer(
            make_transform(parent="/map", child="/pelvis")))
        r.current_pose()

    def test_a_map_mismatch_is_refused_and_preserves_the_file(self):
        store.add(self.path, OTHER, x=9.0, y=9.0, yaw=0.0,
                  recorded_at="t", stamp=1.0, name="from_other_map")
        before = open(self.path, encoding="utf-8").read()
        self._recorder().save()                 # identity is IDENT, file holds OTHER
        self.assertEqual(open(self.path, encoding="utf-8").read(), before)

    def test_the_recorder_never_builds_anything_that_can_move_the_robot(self):
        src = open(RECORDER, encoding="utf-8").read()
        code = "\n".join(line for line in src.splitlines()
                         if not line.strip().startswith("#"))
        body = code.split('"""', 2)[-1]          # drop the module docstring
        for forbidden in ("rospy.Publisher", "move_base_simple", "enable_motion",
                          "Twist", "cmd_vel", "NavigateToObject"):
            self.assertNotIn(forbidden, body,
                             "record_waypoints must not reference %r" % forbidden)

    def test_no_root_keyboard_access(self):
        """Keys come from this terminal's stdin, not a privileged device.

        The scan is over the CODE only: the module docstring names
        /dev/input/event* and `sudo` precisely to say they are not used.
        """
        src = open(RECORDER, encoding="utf-8").read()
        code = src.split('"""', 2)[-1]
        code = "\n".join(line for line in code.splitlines()
                          if not line.strip().startswith("#"))
        for forbidden in ("import keyboard", "/dev/input", "evdev", "sudo", "pynput",
                          "getpass", "os.setuid"):
            self.assertNotIn(forbidden, code,
                             "record_waypoints must not use %r" % forbidden)
        # And it must use the unprivileged terminal path.
        self.assertIn("termios", code)
        self.assertIn("tty.setcbreak", code)
        self.assertIn("sys.stdin.isatty", code)


# --------------------------------------------------------------- goal mapping
class GotoWaypointGoalMapping(unittest.TestCase):
    def setUp(self):
        self.clock = {"t": NOW}
        self.mod, self.patch = _load(GOTO, "goto_waypoint_under_test", self.clock)
        self.addCleanup(self.patch.stop)

    def test_yaw_becomes_the_right_quaternion(self):
        for deg in (-180.0, -90.0, 0.0, 30.0, 90.0, 180.0):
            yaw = math.radians(deg)
            qx, qy, qz, qw = self.mod.quaternion_from_yaw(yaw)
            self.assertAlmostEqual(qx, 0.0, places=12)
            self.assertAlmostEqual(qy, 0.0, places=12)
            back = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
            self.assertAlmostEqual(math.sin(back), math.sin(yaw), places=9)
            self.assertAlmostEqual(math.cos(back), math.cos(yaw), places=9)

    def test_the_record_maps_into_the_goal_unchanged(self):
        record = {"x": 1.25, "y": -0.5, "yaw": 1.5707963}
        goal = self.mod.build_goal(record, "map", FakeTime(NOW),
                                   pose_cls=FakePoseStamped)
        self.assertEqual(goal.header.frame_id, "map")
        self.assertAlmostEqual(goal.pose.position.x, 1.25, places=9)
        self.assertAlmostEqual(goal.pose.position.y, -0.5, places=9)
        self.assertAlmostEqual(goal.pose.position.z, 0.0, places=12)
        self.assertAlmostEqual(goal.pose.orientation.z,
                               math.sin(1.5707963 / 2.0), places=9)
        self.assertAlmostEqual(goal.pose.orientation.w,
                               math.cos(1.5707963 / 2.0), places=9)

    def test_no_approach_offset_is_applied(self):
        """The goal position equals the record exactly, to the last bit."""
        for record in ({"x": 0.0, "y": 0.0, "yaw": 0.0},
                       {"x": -3.75, "y": 2.125, "yaw": -2.0},
                       {"x": 12.5, "y": -0.001, "yaw": 3.14}):
            goal = self.mod.build_goal(record, "map", FakeTime(NOW),
                                       pose_cls=FakePoseStamped)
            self.assertEqual(goal.pose.position.x, record["x"])
            self.assertEqual(goal.pose.position.y, record["y"])

    def test_the_goal_frame_follows_the_record(self):
        goal = self.mod.build_goal({"x": 1.0, "y": 2.0, "yaw": 0.0}, "map_v2",
                                   FakeTime(NOW), pose_cls=FakePoseStamped)
        self.assertEqual(goal.header.frame_id, "map_v2")

    def test_it_targets_navigations_exact_pose_topic(self):
        self.assertEqual(self.mod.GOAL_TOPIC, "/move_base_simple/goal")

    def test_it_never_uses_the_approach_offset_service(self):
        src = open(GOTO, encoding="utf-8").read()
        body = src.split('"""', 2)[-1]
        code = "\n".join(line for line in body.splitlines()
                         if not line.strip().startswith("#"))
        for forbidden in ("NavigateToObject", "ServiceProxy", "approach_distance"):
            self.assertNotIn(forbidden, code)

    def test_it_does_not_open_the_motion_gate(self):
        src = open(GOTO, encoding="utf-8").read()
        body = src.split('"""', 2)[-1]
        code = "\n".join(line for line in body.splitlines()
                         if not line.strip().startswith("#"))
        self.assertNotIn("SetBool", code)
        # It may READ the gate state; it must not call the service.
        self.assertNotIn('"/navigation/enable_motion"', code)

    def test_the_status_contract_matches_navigations_own(self):
        """g1_slam cannot import g1_navigation (that would be a cycle), so the
        ACTIVE/TERMINAL sets are mirrored -- and must not drift."""
        nav_client = os.path.join(HERE, "..", "..", "navigation", "scripts",
                                  "nav_client.py")
        if not os.path.isfile(nav_client):
            self.skipTest("navigation/scripts/nav_client.py not present")
        text = open(nav_client, encoding="utf-8").read()
        for state in self.mod.ACTIVE:
            self.assertIn('"%s"' % state, text,
                          "%s is not in navigation's ACTIVE set" % state)
        for state in self.mod.TERMINAL:
            self.assertIn('"%s"' % state, text)
        self.assertIn("ACTIVE = (", text)
        self.assertIn("TERMINAL = (", text)


class GotoWaypointRecordLookup(unittest.TestCase):
    def setUp(self):
        self.clock = {"t": NOW}
        self.mod, self.patch = _load(GOTO, "goto_waypoint_lookup", self.clock)
        self.addCleanup(self.patch.stop)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "standing_waypoints.yaml")
        store.add(self.path, IDENT, x=1.25, y=-0.5, yaw=1.5707963,
                  recorded_at="2026-10-16T10:22:31", stamp=NOW, name="handover_01")

    def test_a_saved_record_reloads_after_a_restart(self):
        """Nothing in-memory: the file is the whole state."""
        ident, key, record = store.get(self.path, "handover_01")
        self.assertEqual(key, "handover_01")
        self.assertTrue(store.same_map(ident, IDENT))
        goal = self.mod.build_goal(record, record["map_frame"], FakeTime(NOW),
                                   pose_cls=FakePoseStamped)
        self.assertAlmostEqual(goal.pose.position.x, 1.25, places=4)
        self.assertEqual(goal.header.frame_id, "map")

    def test_live_identity_is_read_from_the_manifest(self):
        with open(os.path.join(self.tmp.name, "manifest.yaml"), "w",
                  encoding="utf-8") as fh:
            fh.write("map_name: g1_map_20261016_101500\n"
                     "created_at: '20261016_101500'\n")
        ident = self.mod.live_identity(self.tmp.name, "map", "pelvis")
        self.assertTrue(store.same_map(ident, IDENT))

    def test_a_missing_manifest_is_refused(self):
        with self.assertRaises(RuntimeError) as ctx:
            self.mod.live_identity(self.tmp.name, "map", "pelvis")
        self.assertIn("no manifest", str(ctx.exception))

    def test_a_mismatched_manifest_is_detected(self):
        with open(os.path.join(self.tmp.name, "manifest.yaml"), "w",
                  encoding="utf-8") as fh:
            fh.write("map_name: other_map\ncreated_at: '20261017_090000'\n")
        live = self.mod.live_identity(self.tmp.name, "map", "pelvis")
        file_ident, _wps = store.load(self.path)
        self.assertFalse(store.same_map(file_ident, live))
        self.assertIn("other_map", store.describe_mismatch(file_ident, live))


if __name__ == "__main__":
    unittest.main()
