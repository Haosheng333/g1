#!/usr/bin/env python3
"""Grasp an object at a typed torso_link position, no camera needed.

Thin CLI over the g1_arm_simple node (arm_node_abs.py). Coordinates are in
torso_link: x forward, y left, z up, metres.

  rosrun arm_grasp grasp_point.py plan  0.33 -0.08 0.05   # plan only, never moves
  rosrun arm_grasp grasp_point.py grasp 0.33 -0.08 0.05   # plan, confirm, execute
  rosrun arm_grasp grasp_point.py release --arm right     # open one hand
  rosrun arm_grasp grasp_point.py home                    # open both hands, arms home,
                                                          # hand arms back to the robot

`grasp` runs pregrasp -> hand pre_grasp -> grasp -> hand grasp_5f -> lift.
Without a depth stream the voxel A* plans WITHOUT obstacles, so the table is
invisible to it.
"""

import argparse
import sys

import rospy
from std_srvs.srv import Trigger, TriggerRequest

from arm_grasp.srv import (
    ExecuteGraspFromPoint,
    ExecuteGraspFromPointRequest,
    PlanGraspFromPoint,
    PlanGraspFromPointRequest,
)

NODE = "/g1_arm_simple"


def _call(name, srv_type, req, wait=10.0):
    rospy.wait_for_service(name, timeout=wait)
    return rospy.ServiceProxy(name, srv_type)(req)


DEFAULT_ARM = "left"


def _pick_arm(arm, y):
    return arm or DEFAULT_ARM


def _point_from_perception(label, index, wait=10.0):
    """Ask /perception/get_object_position for the object; returns (x, y, z) or None."""
    from perception.srv import GetObjectPosition, GetObjectPositionRequest  # DIP perception overlay

    name = "/perception/get_object_position"
    try:
        resp = _call(name, GetObjectPosition, GetObjectPositionRequest(label=label, index=index), wait)
    except rospy.ROSException as e:
        print("perception: service %s not available (%s); is perception.launch running?" % (name, e))
        return None
    if not resp.success:
        print("perception: no %r at index %d: %s" % (label, index, resp.message))
        return None
    print("perception: %s at (%.3f, %.3f, %.3f) %s  [%s]" % (label, resp.x, resp.y, resp.z, resp.frame_id, resp.message))
    return resp.x, resp.y, resp.z


def _fmt(values):
    return "[" + ", ".join("%.3f" % v for v in values) + "]"


def _plan(arm, x, y, z):
    req = PlanGraspFromPointRequest(
        arm=arm, x=x, y=y, z=z,
        pre_to_target_waypoint_count=0,
        astar_override=PlanGraspFromPointRequest.ASTAR_USE_DEFAULT,
    )
    resp = _call(NODE + "/plan_grasp_from_point", PlanGraspFromPoint, req)
    print("plan %s arm at (%.3f, %.3f, %.3f): %s"
          % (arm, x, y, z, "OK" if resp.success else "FAILED"))
    print("  message:        %s" % resp.message)
    if resp.success:
        print("  pregrasp pose:  %s" % _fmt(resp.pregrasp_pose))
        print("  grasp pose:     %s" % _fmt(resp.grasp_pose))
        print("  pre_path:       %d waypoints" % resp.pre_waypoint_count)
        print("  target_path:    %d waypoints" % resp.target_waypoint_count)
    return resp.success


def _grasp(arm, x, y, z, timeout, assume_yes):
    if not _plan(arm, x, y, z):
        return False
    if not assume_yes:
        answer = input("Execute the grasp with the %s arm? [y/N] " % arm)
        if answer.strip().lower() not in ("y", "yes"):
            print("aborted, nothing moved")
            return False
    req = ExecuteGraspFromPointRequest(
        arm=arm, x=x, y=y, z=z, wait=True, timeout=timeout)
    resp = _call(NODE + "/execute_grasp_from_point", ExecuteGraspFromPoint, req)
    print("grasp: %s - %s" % ("OK" if resp.success else "FAILED", resp.message))
    return resp.success


def _trigger(name):
    resp = _call(name, Trigger, TriggerRequest())
    print("%s: %s - %s" % (name, "OK" if resp.success else "FAILED", resp.message))
    return resp.success


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for cmd in ("plan", "grasp"):
        p = sub.add_parser(cmd)
        p.add_argument("x", type=float, nargs="?", default=None,
                       help="torso_link metres; omit x y z to use the perception service")
        p.add_argument("y", type=float, nargs="?", default=None)
        p.add_argument("z", type=float, nargs="?", default=None)
        p.add_argument("--label", default="bottle", help="perception label when no point is given")
        p.add_argument("--index", type=int, default=0, help="perception match index when no point is given")
        p.add_argument("--arm", choices=("left", "right"),
                       help="default: left if y > 0, else right")
        if cmd == "grasp":
            p.add_argument("--timeout", type=float, default=6.0,
                           help="per-move timeout passed to the node (s)")
            p.add_argument("--yes", action="store_true",
                           help="skip the confirmation prompt")
    p = sub.add_parser("release")
    p.add_argument("--arm", choices=("left", "right"), required=True)
    sub.add_parser("home")
    args = ap.parse_args(rospy.myargv(argv)[1:])

    rospy.init_node("grasp_point", anonymous=True)
    if args.cmd in ("plan", "grasp"):
        given = [v for v in (args.x, args.y, args.z) if v is not None]
        if len(given) not in (0, 3):
            ap.error("give all three of x y z, or none to use perception")
        if not given:
            point = _point_from_perception(args.label, args.index)
            if point is None:
                return 1
            args.x, args.y, args.z = point
    if args.cmd == "plan":
        ok = _plan(_pick_arm(args.arm, args.y), args.x, args.y, args.z)
    elif args.cmd == "grasp":
        ok = _grasp(_pick_arm(args.arm, args.y), args.x, args.y, args.z,
                    args.timeout, args.yes)
    elif args.cmd == "release":
        ok = _trigger("/g1_hand_%s/release" % args.arm)
    else:
        ok = (_trigger(NODE + "/GoHome")
              and _trigger(NODE + "/release_arm_control"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
