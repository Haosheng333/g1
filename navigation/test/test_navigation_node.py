"""Offline test of navigation_node's state machine: stubs rospy/tf/ROS messages and
simulates the robot.  No ROS master needed.

    cd ~/Workspace/DIP/navigation/test && python3 -m unittest test_navigation_node -v

It stubs the ROS modules unconditionally, so run it with plain python3 (not under
catkin).  Needs the real scripts/ (global_planner.py etc.) next to this folder.
"""
import math
import os
import sys
import tempfile
import types
import unittest

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts")
sys.path.insert(0, SCRIPTS)


# --------------------------------------------------------------------- stubs
class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Vec(_Obj):
    def __init__(self):
        super().__init__(x=0.0, y=0.0, z=0.0)


class _Quat(_Obj):
    def __init__(self):
        super().__init__(x=0.0, y=0.0, z=0.0, w=0.0)


def _install_stubs():
    world = _Obj(now=0.0, params={}, published={}, pose=None, tf_ok=True, svc=None)

    rospy = types.ModuleType("rospy")
    rospy.ROSInterruptException = type("ROSInterruptException", (Exception,), {})
    rospy.init_node = lambda *a, **k: None
    rospy.get_param = lambda name, default=None: world.params.get(name, default)
    rospy.on_shutdown = lambda f: None
    rospy.Subscriber = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    for lvl in ("loginfo", "logwarn", "logerr", "logfatal", "logerr_throttle"):
        setattr(rospy, lvl, lambda *a, **k: None)

    class Time:
        def __init__(self, t=0.0):
            self.t = t

        @staticmethod
        def now():
            return Time(world.now)

        def to_sec(self):
            return self.t

    rospy.Time = Time

    class Publisher:
        def __init__(self, topic, *a, **k):
            self.topic = topic
            world.published.setdefault(topic, [])

        def publish(self, msg):
            world.published[self.topic].append(msg)

    rospy.Publisher = Publisher

    def Service(name, typ, handler):
        world.svc = handler

    rospy.Service = Service

    tf = types.ModuleType("tf")
    tf.LookupException = type("LookupException", (Exception,), {})
    tf.ConnectivityException = type("ConnectivityException", (Exception,), {})
    tf.ExtrapolationException = type("ExtrapolationException", (Exception,), {})

    class TransformListener:
        def lookupTransform(self, target, source, t):
            if not world.tf_ok or world.pose is None:
                raise tf.LookupException("no tf")
            x, y, yaw = world.pose
            return (x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

        def transformPose(self, frame, ps):
            raise tf.LookupException("no tf")

    tf.TransformListener = TransformListener
    tf.transformations = types.SimpleNamespace(
        euler_from_quaternion=lambda q: (0.0, 0.0, 2 * math.atan2(q[2], q[3])))

    geometry = types.ModuleType("geometry_msgs.msg")

    class Twist(_Obj):
        def __init__(self):
            super().__init__(linear=_Vec(), angular=_Vec())

    class Header(_Obj):
        def __init__(self):
            super().__init__(frame_id="", stamp=0)

    class Pose(_Obj):
        def __init__(self):
            super().__init__(position=_Vec(), orientation=_Quat())

    class PoseStamped(_Obj):
        def __init__(self):
            super().__init__(header=Header(), pose=Pose())

    geometry.Twist, geometry.PoseStamped = Twist, PoseStamped

    nav = types.ModuleType("nav_msgs.msg")
    nav.OccupancyGrid = type("OccupancyGrid", (_Obj,), {})
    nav.Path = type("Path", (_Obj,), {"__init__": lambda s: _Obj.__init__(s, header=Header(), poses=[])})

    std = types.ModuleType("std_msgs.msg")
    std.String = lambda data="": _Obj(data=data)
    std.Bool = lambda data=False: _Obj(data=data)
    std.Empty = lambda: _Obj()

    srv = types.ModuleType("g1_navigation.srv")
    srv.NavigateToObject = object
    srv.NavigateToObjectResponse = lambda success, message: _Obj(success=success, message=message)

    for name, mod in {
        "rospy": rospy, "tf": tf, "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": geometry, "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": nav, "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": std,
        "g1_navigation": types.ModuleType("g1_navigation"), "g1_navigation.srv": srv,
    }.items():
        sys.modules[name] = mod
    return world, PoseStamped, nav.OccupancyGrid


WORLD, PoseStamped, OccupancyGrid = _install_stubs()
import navigation_node as nn  # noqa: E402  (after stubs)


# ---------------------------------------------------------------- test world
def make_map(n=60, res=0.1, table=(3.0, 3.0)):
    g = OccupancyGrid()
    g.info = _Obj(width=n, height=n, resolution=res, origin=_Obj(position=_Vec()))
    data = [0] * (n * n)
    for i in range(n):
        for j in range(n):
            if i in (0, n - 1) or j in (0, n - 1):
                data[j * n + i] = 100
    tx, ty = int(table[0] / res), int(table[1] / res)
    for i in range(tx - 2, tx + 3):          # 0.5 m table
        for j in range(ty - 2, ty + 3):
            data[j * n + i] = 100
    g.data = tuple(data)
    return g


class NavTest(unittest.TestCase):
    DT = 0.1

    def setUp(self):
        WORLD.now, WORLD.pose, WORLD.tf_ok = 0.0, (1.0, 1.0, 0.0), True
        WORLD.published.clear()
        self.sem = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        self.sem.write("waypoints:\n  table: {x: 3.0, y: 3.0, yaw: 0.0}\n")
        self.sem.close()
        WORLD.params = {"~semantic_file": self.sem.name, "~use_localization_gate": True}

    def tearDown(self):
        os.unlink(self.sem.name)

    def node(self, **params):
        WORLD.params.update({"~" + k: v for k, v in params.items()})
        n = nn.NavigationNode()
        n.map_cb(make_map())
        return n

    def step(self, n, seconds):
        for _ in range(int(round(seconds / self.DT))):
            WORLD.now += self.DT
            n.tick()
            if n.status is not None:
                n.status_pub.publish(nn.String(data=n.status))
            cmds = WORLD.published.get("/navigation/cmd_vel", [])
            if cmds and WORLD.pose is not None:
                c = cmds[-1]
                x, y, yaw = WORLD.pose
                WORLD.pose = (x + c.linear.x * math.cos(yaw) * self.DT,
                              y + c.linear.x * math.sin(yaw) * self.DT,
                              yaw + c.angular.z * self.DT)

    def statuses(self):
        out = []
        for m in WORLD.published.get("/navigation/status", []):
            if not out or out[-1] != m.data:
                out.append(m.data)
        return out

    @staticmethod
    def goal_msg(x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return m

    # ------------------------------------------------------------------ tests
    def test_rviz_goal_success_with_yaw(self):
        n = self.node()
        n.state_cb(nn.Bool(data=True))
        n.goal_cb(self.goal_msg(2.0, 1.0, 1.0))
        self.assertEqual(n.status, "PLANNING")          # published immediately
        self.step(n, 40)
        self.assertEqual(self.statuses(), ["PLANNING", "FOLLOWING", "SUCCESS"])
        x, y, yaw = WORLD.pose
        self.assertLess(math.hypot(x - 2.0, y - 1.0), 0.10)
        self.assertLess(abs(math.atan2(math.sin(yaw - 1.0), math.cos(yaw - 1.0))), 0.1)
        self.assertEqual(n.status, "SUCCESS")           # terminal state keeps being republished
        self.assertTrue(WORLD.published["/navigation/global_path"])
        self.assertIsNone(n.goal)

    def test_goal_suspended_then_resumed_when_localized(self):
        n = self.node()
        n.goal_cb(self.goal_msg(2.0, 1.0, 0.0))         # NOT localized yet
        self.step(n, 2.0)
        self.assertEqual(n.status, "PLANNING")
        self.assertEqual(n.internal, "RELOCALIZING")
        self.step(n, 2.0)                               # t=4 s > 3 s still -> sweeping
        self.assertGreater(abs(WORLD.published["/navigation/cmd_vel"][-1].angular.z), 0.3)
        self.assertNotIn("FAILURE", self.statuses())
        n.state_cb(nn.Bool(data=True))
        WORLD.pose = (1.0, 1.0, 0.0)
        self.step(n, 40)
        self.assertEqual(self.statuses()[-1], "SUCCESS")
        self.assertNotIn("FAILURE", self.statuses())

    def test_sweep_is_bounded(self):
        n = self.node()
        # integrate the open-loop sweep: yaw must stay within +-amp (plus one tick)
        yaw, lo, hi = 0.0, 0.0, 0.0
        for k in range(int(60 / self.DT)):
            t = k * self.DT - 3.0
            yaw += n._sweep_wz(t) * self.DT
            lo, hi = min(lo, yaw), max(hi, yaw)
        self.assertLess(hi, 0.52 + 0.10)
        self.assertGreater(lo, -0.52 - 0.10)

    def test_gate_timeout_gives_failure(self):
        n = self.node()
        n.goal_cb(self.goal_msg(2.0, 1.0, 0.0))
        self.step(n, 61)
        self.assertEqual(self.statuses(), ["PLANNING", "FAILURE"])
        self.assertIsNone(n.goal)

    def test_localization_lost_midway_suspends(self):
        n = self.node()
        n.state_cb(nn.Bool(data=True))
        n.goal_cb(self.goal_msg(4.0, 1.0, 0.0))
        self.step(n, 5)
        self.assertEqual(n.status, "FOLLOWING")
        n.state_cb(nn.Bool(data=False))
        self.step(n, 1)
        self.assertEqual(n.status, "PLANNING")
        self.assertIsNotNone(n.goal)

    def test_semantic_service(self):
        n = self.node()
        n.state_cb(nn.Bool(data=True))
        resp = WORLD.svc(_Obj(class_name="Table", approach_distance=0.6))
        self.assertTrue(resp.success, resp.message)
        self.assertEqual(n.status, "PLANNING")
        self.step(n, 60)
        self.assertEqual(self.statuses(), ["PLANNING", "FOLLOWING", "SUCCESS"])
        x, y, yaw = WORLD.pose
        d = math.hypot(x - 3.0, y - 3.0)
        self.assertTrue(0.4 < d < 1.0, d)                       # stands in front of it
        facing = math.atan2(3.0 - y, 3.0 - x)
        self.assertLess(abs(math.atan2(math.sin(yaw - facing), math.cos(yaw - facing))), 0.1)

    def test_semantic_unknown_class(self):
        n = self.node()
        resp = WORLD.svc(_Obj(class_name="sofa", approach_distance=0.8))
        self.assertFalse(resp.success)
        self.assertIn("table", resp.message)
        self.assertIsNone(n.status)                             # nothing published

    def test_unreachable_goal_recovers_then_fails(self):
        n = self.node()
        n.state_cb(nn.Bool(data=True))
        n.goal_cb(self.goal_msg(3.0, 3.0, 0.0))                 # inside the table
        self.step(n, 10)
        st = self.statuses()
        self.assertIn("RECOVERING", st)
        self.assertEqual(st[-1], "FAILURE")

    def test_cancel_stops_robot_and_reports_failure(self):
        n = self.node()
        n.state_cb(nn.Bool(data=True))
        n.goal_cb(self.goal_msg(4.0, 1.0, 0.0))
        self.step(n, 3)
        self.assertEqual(n.status, "FOLLOWING")
        n.cancel_cb(nn.Empty())
        self.assertEqual(n.status, "FAILURE")
        self.assertIsNone(n.goal)
        last = WORLD.published["/navigation/cmd_vel"][-1]
        self.assertEqual((last.linear.x, last.angular.z), (0.0, 0.0))

    def test_saw_active_guard_scenario(self):
        """Previous goal's SUCCESS is still on the topic when a new goal arrives;
        the new goal must flip the status to PLANNING before anything else."""
        n = self.node()
        n.state_cb(nn.Bool(data=True))
        n.goal_cb(self.goal_msg(1.5, 1.0, 0.0))
        self.step(n, 20)
        self.assertEqual(n.status, "SUCCESS")
        n.goal_cb(self.goal_msg(2.5, 1.0, 0.0))
        self.assertEqual(n.status, "PLANNING")


if __name__ == "__main__":
    unittest.main()
