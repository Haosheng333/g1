#!/usr/bin/env python3
"""Live check of sections A and B against the RUNNING stack.

Start first (separate shells, `source ~/Workspace/DIP/g1_ws/devel/setup.bash --extend`):
    roslaunch g1_slam localization.launch map_name:=<name>
    roslaunch g1_navigation navigation.launch map_name:=<name>        # motion stays disabled
Then:
    rosrun g1_navigation ab_live_check.py        (or: python3 ~/Workspace/DIP/tools/ab_live_check.py)

It NEVER moves the robot: it refuses to run goal tests if /navigation/motion_enabled is True,
and cancels every goal it sends.  Exit code 0 only if every check passed.
Options:  --semantic PATH   semantic.yaml to test (default: slam/maps/current/semantic.yaml)
          --localize-timeout S   how long to wait for LOCALIZED (default 120)
"""
import argparse
import math
import os
import sys
import time

import rospkg
import rospy
import rosservice
import rostopic
import tf2_ros
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, Empty, String

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok))
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    return ok


def wait_msg(topic, typ, timeout):
    try:
        return rospy.wait_for_message(topic, typ, timeout=timeout)
    except rospy.ROSException:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semantic", default=os.path.join(rospkg.RosPack().get_path("g1_slam"),
                                                       "maps", "current", "semantic.yaml"))
    ap.add_argument("--localize-timeout", type=float, default=120.0)
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("ab_live_check", anonymous=True)

    # ------------------------------------------------------------ B: localization
    print("== B: SLAM / localization")
    t0 = time.time()
    state = None
    while time.time() - t0 < args.localize_timeout:
        m = wait_msg("/slam/manager/state", Bool, 2.0)
        state = bool(m.data) if m is not None else None
        if state:
            break
        st = wait_msg("/slam/relocalizer/status", String, 1.0)
        print("   waiting for localization... %s" % (st.data if st else "no status yet"))
    check("B1 /slam/manager/state is True (localized)", state is True,
          "(state=%s after %.0f s)" % (state, time.time() - t0))

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)
    try:
        t = buf.lookup_transform("map", "camera_init", rospy.Time(0), rospy.Duration(3.0))
        check("B2 TF map -> camera_init exists", True)
    except Exception as exc:  # noqa: BLE001
        t = None
        check("B2 TF map -> camera_init exists", False, str(exc))
    try:
        p = buf.lookup_transform("map", "pelvis", rospy.Time(0), rospy.Duration(3.0))
        q = p.transform.rotation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        check("B3 map -> pelvis is z-UP (|roll|,|pitch| < 15 deg)",
              abs(math.degrees(roll)) < 15 and abs(math.degrees(pitch)) < 15,
              "roll=%.1f pitch=%.1f deg  z=%.2f m" % (math.degrees(roll), math.degrees(pitch),
                                                       p.transform.translation.z))
    except Exception as exc:  # noqa: BLE001
        check("B3 map -> pelvis TF (needs real /joint_states with waist_yaw_joint)", False, str(exc))

    grid = wait_msg("/map", OccupancyGrid, 5.0)
    if grid is None:
        check("B4 /map published", False)
    else:
        vals = set(grid.data)
        check("B4 /map in frame 'map' with tier values", grid.header.frame_id == "map" and
              vals <= {-1, 0, 33, 66, 100} and {0, 100} <= vals,
              "frame=%s values=%s %dx%d" % (grid.header.frame_id, sorted(vals), grid.info.width, grid.info.height))
        free = sum(1 for v in grid.data if v == 0)
        check("B5 walkable area is non-trivial (>1 m^2)", free * grid.info.resolution ** 2 > 1.0,
              "%.1f m^2" % (free * grid.info.resolution ** 2))

    # ------------------------------------------------------------ A: interfaces
    print("== A: navigation interfaces")
    types = {"/navigation/navigate": "g1_navigation/NavigateToObject",
             "/navigation/enable_motion": "std_srvs/SetBool"}
    for srv, typ in types.items():
        try:
            check("A0 service %s" % srv, rosservice.get_service_type(srv) == typ)
        except Exception as exc:  # noqa: BLE001
            check("A0 service %s" % srv, False, str(exc))
    for topic, typ in {"/navigation/status": "std_msgs/String", "/navigation/global_path": "nav_msgs/Path",
                       "/navigation/cmd_vel": "geometry_msgs/Twist", "/navigation/cancel": "std_msgs/Empty"}.items():
        got, _, _ = rostopic.get_topic_type(topic)
        check("A0 topic %s" % topic, got == typ, "(%s)" % got)

    enabled = wait_msg("/navigation/motion_enabled", Bool, 3.0)
    if enabled is None or enabled.data:
        check("A0 motion is DISABLED (required for this check)", False,
              "motion_enabled=%s -> run: rosservice call /navigation/enable_motion \"data: false\"" %
              (None if enabled is None else enabled.data))
        return finish()
    check("A0 motion is DISABLED (required for this check)", True)

    from g1_navigation.srv import NavigateToObject

    navigate = rospy.ServiceProxy("/navigation/navigate", NavigateToObject)
    cancel = rospy.Publisher("/navigation/cancel", Empty, queue_size=1)
    statuses = []
    rospy.Subscriber("/navigation/status", String, lambda m: statuses.append(m.data) if not statuses or statuses[-1] != m.data else None)
    loco = []
    rospy.Subscriber("/locomotion/cmd_vel", Twist, lambda m: loco.append(m))
    rospy.sleep(1.0)

    try:
        r = navigate(class_name="definitely_not_an_object", approach_distance=0.6)
        check("A1 unknown class is rejected with a message", (not r.success) and bool(r.message), r.message)
    except Exception as exc:  # noqa: BLE001
        check("A1 unknown class is rejected", False, str(exc))

    try:
        with open(args.semantic) as f:
            names = list((yaml.safe_load(f) or {}).get("waypoints", {}).keys())
    except Exception as exc:  # noqa: BLE001
        names = []
        print("   cannot read %s: %s" % (args.semantic, exc))
    check("A2 semantic.yaml has real entries (table, sofa expected by the deliver task)",
          bool(names) and {"table", "sofa"} <= {n.lower() for n in names}, "found %s" % names)

    for name in names:
        del statuses[:]
        del loco[:]
        try:
            r = navigate(class_name=name, approach_distance=0.6)
        except Exception as exc:  # noqa: BLE001
            check("A3 navigate('%s') call" % name, False, str(exc))
            continue
        # The topic re-publishes the PREVIOUS goal's terminal state until the new goal is
        # processed (the reason for the doc's saw_active guard): only look from the first
        # PLANNING onwards.
        def seq():
            return statuses[statuses.index("PLANNING"):] if "PLANNING" in statuses else []

        saw_planning = "PLANNING" in statuses[:3]
        deadline = time.time() + 20.0
        while time.time() < deadline and "FOLLOWING" not in seq() and "FAILURE" not in seq():
            rospy.sleep(0.2)
        path = wait_msg("/navigation/global_path", Path, 1.0)
        ok = r.success and "FOLLOWING" in seq() and path is not None and len(path.poses) > 1
        check("A3 navigate('%s') -> approach pose found and path planned" % name, ok,
              "status sequence=%s  path_points=%s  msg=%s" % (seq(), len(path.poses) if path else None, r.message))
        check("A3 '%s': PLANNING published immediately (saw_active guard)" % name, saw_planning)
        rospy.sleep(2.0)
        check("A4 '%s': /locomotion/cmd_vel stays silent while motion disabled" % name, len(loco) == 0,
              "(%d messages)" % len(loco))
        cancel.publish(Empty())
        rospy.sleep(1.0)
        check("A5 '%s': cancel -> FAILURE reported" % name, bool(statuses) and statuses[-1] == "FAILURE", str(statuses[-2:]))

    return finish()


def finish():
    bad = [n for n, ok in RESULTS if not ok]
    print("\nRESULT: %s (%d checks)" % ("ALL PASS" if not bad else "FAILED: %d" % len(bad), len(RESULTS)))
    for n in bad:
        print("   FAILED:", n)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
