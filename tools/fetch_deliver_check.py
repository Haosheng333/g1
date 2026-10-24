#!/usr/bin/env python3
"""Readiness checklist for the fetch-and-deliver pipeline, then the dry run.

    tools/fetch_deliver_check.py --side left --map-name lab_map_DIP
    tools/fetch_deliver_check.py --until-ready        # re-run every 5 s until READY (or --timeout)
    tools/fetch_deliver_check.py --no-dryrun          # checklist only
    tools/fetch_deliver_check.py --no-perception-call # do not query the camera

Every line is PASS / WAIT / FAIL with the reason, in the order the route needs
things: relocalization first (slam_manager and relocalizer status, fitness),
then navigation, perception, arm, hands, handover, orchestrator and the voice
node. WAIT is a latched topic that has not arrived yet or a localization still
searching: usually a matter of seconds, or of the robot not being connected.

The last line is the verdict: READY TO SPEAK (every item PASS, the orchestrator
in the expected motion mode, the voice node listening and dispatching for real)
or NOT READY with what is missing. Nothing here moves the robot: the only
services called for real are perception's (a camera read) and the
orchestrator's with dry_run: true.
"""
import argparse
import os
import subprocess
import sys
import time

import rospy
import rosnode
import rosservice
import rostopic
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import JointState

SENTENCE = "go to the desk, pick up a can, go to the chair, and pass me, over"

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


def check_relocalization(timeout):
    """slam_manager's verdict, the relocalizer's own status and its fit quality."""
    localized = check_latched_bool("localization", "/slam/manager/state", timeout)
    try:
        status = rospy.wait_for_message("/slam/manager/status", String, timeout=2.0).data
        report(PASS if localized else WAIT, "slam_manager status", status)
    except rospy.ROSException:
        report(WAIT if publishers("/slam/manager/status") else FAIL, "slam_manager status",
               "/slam/manager/status not received" if publishers("/slam/manager/status")
               else "/slam/manager/status has no publisher (localization.launch not running?)")
    try:
        status = rospy.wait_for_message("/slam/relocalizer/status", String, timeout=2.0).data
        report(PASS if localized else WAIT, "relocalizer status", status)
    except rospy.ROSException:
        report(WAIT if publishers("/slam/relocalizer/status") else FAIL, "relocalizer status",
               "/slam/relocalizer/status not received" if publishers("/slam/relocalizer/status")
               else "/slam/relocalizer/status has no publisher")
    try:
        fit = rospy.wait_for_message("/slam/relocalizer/fitness", Float32, timeout=2.0).data
        report(PASS if localized else WAIT, "relocalizer fitness", "%.3f (scan-to-map fit; lower is better)" % fit)
    except rospy.ROSException:
        report(WAIT, "relocalizer fitness", "/slam/relocalizer/fitness not received (published while matching)")
    return localized


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


def subscribers(topic):
    try:
        _pubs, subs = rostopic.get_topic_list()
        for name, _type, nodes in subs:
            if name == topic:
                return list(nodes)
    except Exception:  # noqa: BLE001
        pass
    return []


def check_voice(expect_voice):
    """The voice node: up, listening on a microphone, and publishing for real."""
    try:
        nodes = rosnode.get_node_names()
    except Exception as exc:  # noqa: BLE001
        report(FAIL, "voice node", "cannot list nodes: %s" % exc)
        return False
    subs = subscribers("/voice/command")
    if any("orchestrator" in n for n in subs):
        report(PASS, "orchestrator listens to voice", "/voice/command subscribed by %s" % ", ".join(subs))
    else:
        report(FAIL, "orchestrator listens to voice",
               "/voice/command has no orchestrator subscriber (orchestrator not running, or the "
               "g1_voice_control messages are not built)")
    if "/voice_control_node" not in nodes:
        if expect_voice:
            report(FAIL, "voice node", "/voice_control_node is not running (VOICE=true starts it)")
            return False
        report(WAIT, "voice node", "not running; commands go through rosservice only")
        return False
    mode = str(rospy.get_param("/voice_control_node/input_mode", "microphone"))
    dry = bool(rospy.get_param("/voice_control_node/dry_run_backend", True))
    confirm = str(rospy.get_param("/voice_control_node/confirmation_mode", "") or "confirm")
    device = str(rospy.get_param("/voice_control_node/input_device", "") or "default")
    ok = True
    if mode != "microphone":
        report(WAIT, "voice input", "input_mode=%s (not listening to a microphone)" % mode)
        ok = False
    else:
        report(PASS, "voice input", "microphone device %s, confirmation_mode=%s" % (device, confirm))
    if dry:
        report(WAIT, "voice dispatch", "dry_run_backend=true: commands are recognised but NOT published to the orchestrator")
        ok = False
    else:
        report(PASS, "voice dispatch", "dry_run_backend=false: a confirmed command is published on /voice/command")
    return ok and any("orchestrator" in n for n in subs)


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


def run_once(args):
    del RESULTS[:]
    print("== fetch_deliver readiness  (side=%s map=%s label=%s)  master=%s"
          % (args.side, args.map_name, args.label, os.environ.get("ROS_MASTER_URI", "")))

    print("-- relocalization")
    check_relocalization(args.wait)

    print("-- navigation")
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
    motion_ok = False
    if check_services("orchestrator route", ["/orchestrator/run_fetch_deliver"]):
        try:
            status = rospy.wait_for_message("/orchestrator/status", String, timeout=3.0).data
            motion_on = "MOTION ENABLED" in status
            if args.expect_motion and not motion_on:
                report(WAIT, "orchestrator mode", status + "  (expected MOTION ENABLED; restart with ENABLE_MOTION=true)")
            elif not args.expect_motion and motion_on:
                report(WAIT, "orchestrator mode", status + "  (expected a dry run; restart with ENABLE_MOTION=false)")
            else:
                report(PASS, "orchestrator mode", status)
                motion_ok = True
        except rospy.ROSException:
            report(WAIT, "orchestrator mode", "/orchestrator/status not received")

    print("-- voice")
    voice_ok = check_voice(args.expect_voice)

    if not args.no_dryrun:
        dry_run(args.label)

    counts = {k: sum(1 for s, _i, _d in RESULTS if s == k) for k in (PASS, WAIT, FAIL)}
    print("\n== %d PASS, %d WAIT, %d FAIL" % (counts[PASS], counts[WAIT], counts[FAIL]))
    if counts[FAIL] or counts[WAIT]:
        pending = [i for s_, i, _d in RESULTS if s_ != PASS]
        print("   NOT READY. Still open: " + ", ".join(pending))
        if counts[FAIL]:
            print("   fix every FAIL (the detail names the owning launch / script), then re-run `check`")
        else:
            print("   WAIT items usually clear on their own: `check --until-ready` keeps watching")
        return 1
    if voice_ok and motion_ok and args.expect_motion:
        print("   READY TO SPEAK. Say:  \"%s\"" % SENTENCE)
        print("   then \"confirm\" within 8 s. The robot WILL MOVE.")
    elif voice_ok and motion_ok:
        print("   READY (dry run). Say:  \"%s\"  then \"confirm\"; nothing moves." % SENTENCE)
    else:
        print("   READY for a typed command: rosservice call /orchestrator/run_fetch_deliver "
              "\"{target: table_01, destination: chair_01, label: %s}\"" % args.label)
    return 0


def _bool(text):
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", default="left")
    ap.add_argument("--map-name", default="lab_map_DIP")
    ap.add_argument("--label", default="bottle")
    ap.add_argument("--wait", type=float, default=5.0, help="seconds to wait for each latched topic")
    ap.add_argument("--expect-motion", default="true",
                    help="true: the orchestrator must report MOTION ENABLED to be ready (default)")
    ap.add_argument("--expect-voice", default="true",
                    help="true: the voice node must be listening and dispatching to be ready (default)")
    ap.add_argument("--until-ready", action="store_true", help="repeat every --interval seconds until ready")
    ap.add_argument("--interval", type=float, default=5.0)
    ap.add_argument("--timeout", type=float, default=300.0, help="give up --until-ready after this many seconds")
    ap.add_argument("--no-dryrun", action="store_true")
    ap.add_argument("--no-perception-call", action="store_true")
    args = ap.parse_args(rospy.myargv(sys.argv)[1:])
    args.expect_motion = _bool(args.expect_motion)
    args.expect_voice = _bool(args.expect_voice)

    try:
        rospy.init_node("fetch_deliver_check", anonymous=True)
    except Exception as exc:  # noqa: BLE001
        print("[FAIL] roscore: %s" % exc)
        return 2

    deadline = time.time() + args.timeout
    while True:
        code = run_once(args)
        if code == 0 or not args.until_ready or rospy.is_shutdown():
            return code
        if time.time() >= deadline:
            print("   gave up after %.0fs" % args.timeout)
            return code
        print("   ... re-checking in %.0fs (Ctrl+C to stop)\n" % args.interval)
        time.sleep(args.interval)


if __name__ == "__main__":
    sys.exit(main())
