#!/usr/bin/env python3
"""Navigation acceptance test (doc section 4.4):

    "from any start, the robot reaches a clicked goal around furniture with < 0.10 m error"

Works against the SIMULATOR (default goals fit navigation/sim/sim_room) and against the real robot
(pass --goals-file with poses valid in YOUR map; use a cleared area and keep the e-stop in hand).

    roslaunch g1_navigation sim.launch rviz:=false         # shell 1
    python3 tools/nav_acceptance_test.py --dynamic          # shell 2

Checks, each reported PASS/FAIL (exit code 0 only if all pass):
  * pose goals           SUCCESS, final position error < 0.10 m and yaw error < 0.1 rad (measured from TF)
  * semantic goals       every name in semantic.yaml: SUCCESS, stands ~approach_distance away FACING the object
  * saw_active guard     status goes PLANNING first (never an echo of the previous goal's terminal state)
  * cancel               /navigation/cancel -> FAILURE and the robot stops
  * --dynamic (sim only) a box appears in the way mid-goal: still SUCCESS, zero collisions
  * collisions           /sim/collision_count == 0 at the end (sim only)

--goals-file YAML:   goals: [[x, y, yaw], ...]
"""
import argparse
import math
import os
import sys
import time

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, Int32, String
from std_srvs.srv import Trigger

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "navigation", "scripts"))
from nav_client import ACTIVE, TERMINAL, StatusWatcher  # noqa: E402

SIM_GOALS = [(8.5, 6.5, 1.0), (1.0, 6.0, -1.5), (3.4, 2.0, 3.0), (6.2, 4.0, 0.0), (1.0, 1.0, 0.0)]
XY_TOL, YAW_TOL = 0.10, 0.1
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok))
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    return ok


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Runner:
    def __init__(self):
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self.watch = StatusWatcher()
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        self.cancel_pub = rospy.Publisher("/navigation/cancel", Empty, queue_size=1)
        self.seen = []
        rospy.Subscriber("/navigation/status", String,
                         lambda m: self.seen.append(m.data) if not self.seen or self.seen[-1] != m.data else None)

    def pose(self):
        t = self.buf.lookup_transform("map", "pelvis", rospy.Time(0), rospy.Duration(2.0))
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def send_pose_goal(self, x, y, yaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        del self.seen[:]
        self.goal_pub.publish(g)

    def wait_goal(self, timeout, on_tick=None):
        """Wait for a terminal status that follows an ACTIVE one (the saw_active guard)."""
        saw_active, t0 = False, time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            s = self.watch.latest
            if s in ACTIVE:
                saw_active = True
            elif s in TERMINAL and saw_active:
                return s
            if on_tick:
                on_tick(time.time() - t0)
            rospy.sleep(0.1)
        return "TIMEOUT"

    def settle(self, seconds=1.0):
        rospy.sleep(seconds)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goals-file")
    ap.add_argument("--semantic", default=os.path.join(REPO, "navigation", "sim", "sim_room", "semantic.yaml"))
    ap.add_argument("--approach", default="table=0.6,sofa=0.8", help="name=distance,... for semantic goals")
    ap.add_argument("--dynamic", action="store_true", help="sim only: toggle a box into the way mid-goal")
    ap.add_argument("--timeout", type=float, default=240.0, help="seconds per goal")
    ap.add_argument("--localize-timeout", type=float, default=90.0)
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("nav_acceptance_test", anonymous=True)

    goals = SIM_GOALS
    if args.goals_file:
        with open(args.goals_file) as f:
            goals = [tuple(g) for g in yaml.safe_load(f)["goals"]]
    run = Runner()

    # ------------------------------------------------------------------ preconditions
    try:
        loc = rospy.wait_for_message("/slam/manager/state", Bool, timeout=args.localize_timeout)
        deadline = time.time() + args.localize_timeout
        while not loc.data and time.time() < deadline:
            loc = rospy.wait_for_message("/slam/manager/state", Bool, timeout=5.0)
        check("localized (/slam/manager/state True)", bool(loc.data))
    except rospy.ROSException:
        check("localized (/slam/manager/state True)", False, "no message")
        return finish()
    try:
        run.pose()
        check("TF map -> pelvis available", True)
    except Exception as exc:  # noqa: BLE001
        check("TF map -> pelvis available", False, str(exc))
        return finish()
    rospy.sleep(2.0)

    # ------------------------------------------------------------------ pose goals
    for i, (x, y, yaw) in enumerate(goals, 1):
        run.send_pose_goal(x, y, yaw)
        res = run.wait_goal(args.timeout)
        run.settle()
        px, py, pyaw = run.pose()
        err, yerr = math.hypot(px - x, py - y), abs(wrap(pyaw - yaw))
        check("pose goal %d/%d (%.1f, %.1f, %.2f)" % (i, len(goals), x, y, yaw),
              res == "SUCCESS" and err < XY_TOL and yerr < YAW_TOL,
              "-> %s, error %.3f m (<%.2f), yaw %.3f rad (<%.1f)" % (res, err, XY_TOL, yerr, YAW_TOL))
        check("goal %d: PLANNING came first (saw_active)" % i, bool(run.seen) and run.seen[0] == "PLANNING",
              str(run.seen[:4]))

    # ------------------------------------------------------------------ semantic goals
    from g1_navigation.srv import NavigateToObject
    navigate = rospy.ServiceProxy("/navigation/navigate", NavigateToObject)
    try:
        with open(args.semantic) as f:
            objects = {k.lower(): v for k, v in (yaml.safe_load(f) or {}).get("waypoints", {}).items()}
    except Exception as exc:  # noqa: BLE001
        objects = {}
        check("semantic.yaml readable", False, "%s: %s" % (args.semantic, exc))
    approach = dict((kv.split("=")[0], float(kv.split("=")[1])) for kv in args.approach.split(","))
    for name, obj in objects.items():
        a = approach.get(name, 0.6)
        del run.seen[:]
        run.watch.latest = None
        r = navigate(class_name=name, approach_distance=a)
        res = run.wait_goal(args.timeout) if r.success else "REJECTED"
        run.settle()
        px, py, pyaw = run.pose()
        d = math.hypot(obj["x"] - px, obj["y"] - py)
        facing = abs(wrap(pyaw - math.atan2(obj["y"] - py, obj["x"] - px)))
        check("semantic goal '%s' (approach %.2f m)" % (name, a),
              res == "SUCCESS" and a - 0.15 < d < a + 0.6 and facing < 0.15,
              "-> %s, distance %.2f m, facing error %.3f rad" % (res, d, facing))

    # ------------------------------------------------------------------ cancel
    run.send_pose_goal(*goals[0])
    rospy.sleep(3.0)
    run.cancel_pub.publish(Empty())
    rospy.sleep(1.0)
    p1 = run.pose()
    rospy.sleep(1.5)
    p2 = run.pose()
    check("cancel -> FAILURE and the robot stops", run.watch.latest == "FAILURE" and math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 0.03,
          "(status %s)" % run.watch.latest)

    # ------------------------------------------------------------------ dynamic obstacle (sim)
    if args.dynamic:
        try:
            rospy.wait_for_service("/sim/toggle_obstacle", timeout=3.0)
        except rospy.ROSException:
            check("dynamic obstacle test needs the simulator", False, "/sim/toggle_obstacle missing")
        else:
            toggle = rospy.ServiceProxy("/sim/toggle_obstacle", Trigger)
            run.send_pose_goal(1.0, 1.0, 0.0)
            run.wait_goal(args.timeout)
            fired = []

            def on_tick(t):
                if t > 4.0 and not fired:
                    fired.append(toggle().message)

            run.send_pose_goal(5.5, 1.0, 0.0)               # straight line passes the box at (3.0, 1.0)
            res = run.wait_goal(args.timeout, on_tick)
            if fired and "ON" in fired[0]:
                toggle()                                    # remove it again
            run.settle()
            px, py, _ = run.pose()
            check("dynamic obstacle: re-plans and still arrives", res == "SUCCESS" and math.hypot(px - 5.5, py - 1.0) < XY_TOL,
                  "-> %s, error %.3f m" % (res, math.hypot(px - 5.5, py - 1.0)))

    # ------------------------------------------------------------------ collisions (sim)
    try:
        n = rospy.wait_for_message("/sim/collision_count", Int32, timeout=2.0).data
        check("ground-truth collisions == 0", n == 0, "(%d)" % n)
    except rospy.ROSException:
        print("[skip] /sim/collision_count not published (real robot): watch the robot yourself")
    return finish()


def finish():
    bad = [n for n, ok in RESULTS if not ok]
    print("\nRESULT: %s (%d checks)" % ("ALL PASS" if not bad else "FAILED: %d" % len(bad), len(RESULTS)))
    for n in bad:
        print("   FAILED:", n)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
