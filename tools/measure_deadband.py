#!/usr/bin/env python3
"""ON-ROBOT: measure the G1 walking controller's velocity dead-band and gain.  THE ROBOT WILL WALK.

    python3 tools/measure_deadband.py --yes [--max-v 0.15]

Needs the live TF chain (map -> pelvis), a CLEAR 2 m x 2 m floor, a person at the e-stop, and the
locomotion controller listening on /locomotion/cmd_vel.  It does NOT use cmd_vel_gate: stop navigation
first so nothing else publishes.  For vx, vy and wz it commands increasing steps (each 3 s, then 2 s
stop, direction alternates so the robot stays in place) and measures the achieved velocity from TF.

Prints per-axis: smallest command that really moves the robot (dead-band), achieved/commanded gain,
and suggested  navigation.yaml  values.  The sim shows the planner copes with a 0.03 m/s dead-band and a
0.8 gain; larger values need `nav_limits` min speeds raised.
"""
import argparse
import math
import sys
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Twist


def pose(buf):
    t = buf.lookup_transform("map", "pelvis", rospy.Time(0), rospy.Duration(0.5))
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yes", action="store_true", help="confirm the floor is clear and the e-stop is held")
    ap.add_argument("--max-v", type=float, default=0.15, help="largest vx/vy step (m/s)")
    ap.add_argument("--max-w", type=float, default=0.4, help="largest wz step (rad/s)")
    ap.add_argument("--topic", default="/locomotion/cmd_vel")
    args, _ = ap.parse_known_args(rospy.myargv()[1:])
    if not args.yes:
        sys.exit("refusing to move the robot without --yes (clear floor, e-stop in hand)")
    rospy.init_node("measure_deadband", anonymous=True)
    pub = rospy.Publisher(args.topic, Twist, queue_size=1)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)
    pose(buf)                                   # fail early if TF is missing

    def drive(vx, vy, wz, secs):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        rate = rospy.Rate(20)
        end = time.time() + secs
        while time.time() < end and not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()

    def stop():
        drive(0, 0, 0, 2.0)

    results = {}
    try:
        for axis, top, unit in (("vx", args.max_v, "m/s"), ("vy", args.max_v * 0.6, "m/s"), ("wz", args.max_w, "rad/s")):
            levels = [top * f for f in (0.1, 0.2, 0.3, 0.5, 0.75, 1.0)]
            rows, sign = [], 1.0
            for v in levels:
                stop()
                x0, y0, a0 = pose(buf)
                cmd = [0.0, 0.0, 0.0]
                cmd[("vx", "vy", "wz").index(axis)] = sign * v
                t0 = time.time()
                drive(cmd[0], cmd[1], cmd[2], 3.0)
                x1, y1, a1 = pose(buf)
                dt = time.time() - t0
                dx, dy = x1 - x0, y1 - y0
                if axis == "vx":
                    got = (dx * math.cos(a0) + dy * math.sin(a0)) / dt
                elif axis == "vy":
                    got = (-dx * math.sin(a0) + dy * math.cos(a0)) / dt
                else:
                    got = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)) / dt
                rows.append((v, abs(got)))
                print("%s cmd %+.3f %s -> achieved %+.3f %s" % (axis, sign * v, unit, got, unit))
                sign = -sign
            moving = [v for v, g in rows if g > 0.5 * v and g > 0.01]
            db = min(moving) if moving else float("nan")
            big = [(v, g) for v, g in rows if v >= 0.5 * top]
            gain = sum(g for _, g in big) / sum(v for v, _ in big)
            results[axis] = (db, gain)
    finally:
        stop()
        pub.publish(Twist())
    print("\nRESULT (smallest command that really moves | achieved/commanded gain at >= 50 % of top step)")
    for axis, (db, gain) in results.items():
        print("  %s: dead-band ~ %.3f | gain %.2f" % (axis, db, gain))
    print("\nSimulation tolerance: dead-band <= 0.03 and gain >= 0.8 reach the 0.10 m / 0.1 rad spec.")
    print("If a dead-band exceeds dwa.terminal_limits (0.06) the last 0.3 m cannot be completed: raise terminal_limits/approach_limits in navigation.yaml above the measured dead-band.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
