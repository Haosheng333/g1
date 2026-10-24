#!/usr/bin/env python3
"""Readiness checklist for the fetch-and-deliver pipeline, then the dry run.

    tools/fetch_deliver_check.py --side left --map-name lab_map_DIP
    tools/fetch_deliver_check.py --no-dryrun          # checklist only
    tools/fetch_deliver_check.py --no-perception-call # do not query the camera

Every line is PASS / WAIT / FAIL with the reason, in the order the route needs
things. WAIT is a latched topic that has not arrived yet (localization, joint
states): usually a matter of seconds, or of the robot not being connected.
Nothing here moves the robot: the only service called for real is perception's
(a camera read) and the orchestrator's with dry_run: true.
"""
import argparse
import os
import subprocess
import sys
import time

import rospy
import rosservice
import rostopic
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState

PASS, WAIT, FAIL = "PASS", "WAIT", "FAIL"
RESULTS = []


def report(status, item, detail=""):
    RESULTS.append((status, item, detail))
    print("  [%s] %-44s %s" % (status, item, detail))


def service_present(name):
    try:
        return name in rosservice.get_service_list()
    except Exception as exc:  # noqa: BLE001
        return False


def publishers(topic):
    try:
        pubs, _subs = rostopic.get_topic_list()
        for name, _type, nodes in pubs:
            if name == topic:
                return list(nodes)
    except Exception:  # noqa: BLE001
        pass
    return []


def check_services(item, names):
    missing = [n for n in names if not service_present(n)]
    if missing:
        report(FAIL, item, "missing: " + ", ".join(missing))
    else:
        report(PASS, item, ", ".join(names))
    return not missing


def check_latched_bool(item, topic, timeout):
    try:
        msg = rospy.wait_for_message(topic, Bool, timeout=timeout)
    except rospy.ROSException:
        report(WAIT if publishers(topic) else FAIL, item,
               "%s: %s" % (topic, "no message yet" if publishers(topic) else "no publisher"))
        return False
    if msg.data:
        report(PASS, item, "%s is True" % topic)
    else:
        report(WAIT, item, "%s is False (not localized yet)" % topic)
    return bool(msg.data)


def check_joint_states(timeout):
    try:
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=timeout)
    except rospy.ROSException:
        report(FAIL if not publishers("/joint_states") else WAIT, "joint states",
               "/joint_states: " + ("no publisher (g1_joint_state_bridge)" if not publishers("/joint_states") else "no message"))
        return False
    age = rospy.Time.now().to_sec() - msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0 else 0.0
    if "waist_yaw_joint" not in msg.name:
        report(FAIL, "joint states", "/joint_states has no waist_yaw_joint (%d joints)" % len(msg.name))
        return False
    if age > 0.5:
        report(FAIL, "joint states", "/joint_states stale: %.2fs old" % age)
        return False
    report(PASS, "joint states", "%d joints incl. waist_yaw_joint, %.2fs old" % (len(msg.name), age))
    return True


def check_topic(item, topic, wait_s=0.0):
    pubs = publishers(topic)
    if not pubs:
        report(FAIL, item, "%s has no publisher" % topic)
        return False
    report(PASS, item, "%s from %s" % (topic, ", ".join(pubs)))
    return True


def check_motion_gate():
    try:
        msg = rospy.wait_for_message("/navigation/motion_enabled", Bool, timeout=3.0)
    except rospy.ROSException:
        report(WAIT, "motion gate state", "/navigation/motion_enabled not received")
        return
    report(PASS if not msg.data else WAIT, "motion gate state",
           "gate is %s (the route opens it itself)" % ("SHUT" if not msg.data else "OPEN already"))


def check_perception_call(label):
    from perception.srv import GetObjectPosition
    try:
        rospy.wait_for_service("/perception/get_object_position", timeout=3.0)
        resp = rospy.ServiceProxy("/perception/get_object_position", GetObjectPosition)(label=label, index=0)
    except Exception as exc:  # noqa: BLE001
        report(FAIL, "perception sees a %s" % label, str(exc))
        return
    if resp.success:
        report(PASS, "perception sees a %s" % label,
               "(%.3f, %.3f, %.3f) in %s" % (resp.x, resp.y, resp.z, resp.frame_id))
    else:
        report(WAIT, "perception sees a %s" % label, "not now: %s (fine if the can is not in view yet)" % resp.message)


def check_handover(side):
    ok = check_services("handover_node services",
                        ["/handover/start", "/handover/abort", "/auto_release_monitor/arm",
                         "/auto_release_monitor/disarm"])
    check_topic("handover state topic", "/handover/state")
    arm = str(rospy.get_param("/handover/arm", "") or "")
    if not arm:
        report(FAIL, "handover arm", "/handover/arm not set (handover.launch not running?)")
    elif arm != side:
        report(FAIL, "handover arm", "handover_node is for the %s arm, the orchestrator grasps with the %s" % (arm, side))
    else:
        report(PASS, "handover arm", "%s, same as the orchestrator" % arm)
    check_topic("tactile data for the monitor", "/g1_hand_%s/tactile" % side)
    return ok


def check_standing_waypoints(map_name):
    try:
        from rospkg import RosPack
        from g1_slam import standing_goal
        maps_dir = os.path.join(RosPack().get_path("g1_slam"), "maps")
        for name in ("table_01", "chair_01"):
            key, rec, frame = standing_goal.resolve(maps_dir, map_name, name)
            report(PASS, "standing waypoint %s" % name,
                   "x=%+.2f y=%+.2f yaw=%+.2f in %s (map %s)" % (rec["x"], rec["y"], rec["yaw"], frame, map_name))
    except Exception as exc:  # noqa: BLE001
        report(FAIL, "standing waypoints", str(exc))


def dry_run(label):
    from orchestrator.srv import RunRoute
    print("\n== dry run: /orchestrator/run_fetch_deliver target=table_01 destination=chair_01 label=%s" % label)
    try:
        rospy.wait_for_service("/orchestrator/run_fetch_deliver", timeout=3.0)
        resp = rospy.ServiceProxy("/orchestrator/run_fetch_deliver", RunRoute)(
            route="fetch_deliver", target="table_01", destination="chair_01", label=label,
            start_step="", dry_run=True, event_token="")
    except Exception as exc:  # noqa: BLE001
        report(FAIL, "dry run", str(exc))
        return False
    print("  success=%s dry_run=%s motion_issued=%s steps=%d" % (
        resp.success, resp.dry_run, resp.motion_issued, len(resp.steps_completed)))
    print("  " + resp.message)
    if resp.blockers:
        print("  blockers a REAL run would refuse on:")
        for b in resp.blockers:
            print("    - " + b)
        report(FAIL, "dry run", "%d blocker(s)" % len(resp.blockers))
        return False
    report(PASS, "dry run", "no blockers: a real run is possible (enable_motion:=true)")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", default="left")
    ap.add_argument("--map-name", default="lab_map_DIP")
    ap.add_argument("--label", default="bottle")
    ap.add_argument("--wait", type=float, default=5.0, help="seconds to wait for each latched topic")
    ap.add_argument("--no-dryrun", action="store_true")
    ap.add_argument("--no-perception-call", action="store_true")
    args = ap.parse_args(rospy.myargv(sys.argv)[1:])

    try:
        rospy.init_node("fetch_deliver_check", anonymous=True)
    except Exception as exc:  # noqa: BLE001
        print("[FAIL] roscore: %s" % exc)
        return 2
    print("== fetch_deliver readiness  (side=%s map=%s label=%s)  master=%s"
          % (args.side, args.map_name, args.label, os.environ.get("ROS_MASTER_URI", "")))

    print("-- slam / navigation")
    check_latched_bool("localization", "/slam/manager/state", args.wait)
    check_topic("map", "/map")
    check_joint_states(args.wait)
    check_services("navigation services", ["/navigation/navigate", "/navigation/enable_motion"])
    check_topic("navigation status", "/navigation/status")
    check_motion_gate()
    check_standing_waypoints(args.map_name)

    print("-- perception (driver on PC2 or local)")
    check_topic("camera colour stream", "/camera/color/image_raw")
    check_topic("camera aligned depth", "/camera/aligned_depth_to_color/image_raw")
    if check_services("perception service", ["/perception/get_object_position"]) and not args.no_perception_call:
        check_perception_call(args.label)

    print("-- arm_grasp / hands (PC2)")
    check_services("arm_grasp services", ["/g1_arm_simple/plan_grasp_from_point",
                                          "/g1_arm_simple/execute_grasp_from_point",
                                          "/g1_arm_simple/release_arm_control"])
    check_services("%s hand services" % args.side, ["/g1_hand_%s/%s" % (args.side, n)
                                                      for n in ("pre_grasp", "grasp_5f", "release")])

    print("-- handover")
    check_handover(args.side)

    print("-- orchestrator")
    if check_services("orchestrator route", ["/orchestrator/run_fetch_deliver"]):
        try:
            status = rospy.wait_for_message("/orchestrator/status", String, timeout=3.0).data
            mode = "MOTION ENABLED" if "MOTION" in status and "dry-run" not in status else "dry run"
            report(PASS, "orchestrator mode", status)
        except rospy.ROSException:
            report(WAIT, "orchestrator mode", "/orchestrator/status not received")

    if not args.no_dryrun:
        dry_run(args.label)

    counts = {k: sum(1 for s, _i, _d in RESULTS if s == k) for k in (PASS, WAIT, FAIL)}
    print("\n== %d PASS, %d WAIT, %d FAIL" % (counts[PASS], counts[WAIT], counts[FAIL]))
    if counts[FAIL]:
        print("   fix every FAIL, re-run `check`, then restart the orchestrator with ENABLE_MOTION=true")
        return 1
    if counts[WAIT]:
        print("   WAIT items usually clear on their own; re-run `check`")
        return 1
    print("   ready: ENABLE_MOTION=true tools/fetch_deliver_pipeline.sh restart orchestrator, then run the route")
    return 0


if __name__ == "__main__":
    sys.exit(main())
