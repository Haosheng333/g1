#!/usr/bin/env python3
"""LIVE check of the TF chain timing.  Run with the real stack up (system_bringup navigation.launch).

    rosrun ... or:  python3 tools/check_tf_timing.py [--seconds 20]

Measures, from the running system:
  * /Odometry vs /joint_states stamp skew  -> g1_tf_publisher drops the frame if |skew| > joint_states_timeout (0.5 s).
    A big, constant skew means FAST-LIO2/Livox stamp with the LiDAR clock, not ROS time.
  * rates: /Odometry, /joint_states, /cloud_registered, /obstacle_cloud, /map
  * map->pelvis lookup: success rate and age of the newest transform (navigation uses Time(0))
  * map->camera_init age (relocalizer publishes at 10 Hz)
  * whether `base_link` is in the tree (slam_health_check FAILs if so)
Exit 0 only if every check passes.
"""
import argparse
import sys
import threading
import time

import rospy
import tf2_ros
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import JointState, PointCloud2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=0.5, help="g1_tf.yaml joint_states_timeout")
    args, _ = ap.parse_known_args(rospy.myargv()[1:])
    rospy.init_node("check_tf_timing", anonymous=True)
    lock = threading.Lock()
    cnt = {k: 0 for k in ("odom", "js", "cloud", "obs", "map")}
    skew, odom_vs_now, js_vs_now = [], [], []
    last = {"odom": None, "js": None}

    def odom(m):
        with lock:
            cnt["odom"] += 1
            last["odom"] = m.header.stamp.to_sec()
            odom_vs_now.append(rospy.Time.now().to_sec() - last["odom"])
            if last["js"] is not None:
                skew.append(last["odom"] - last["js"])

    def js(m):
        with lock:
            cnt["js"] += 1
            last["js"] = m.header.stamp.to_sec() or rospy.Time.now().to_sec()
            js_vs_now.append(rospy.Time.now().to_sec() - last["js"])

    def counter(k):
        def f(_m):
            with lock:
                cnt[k] += 1
        return f

    rospy.Subscriber("/Odometry", Odometry, odom, queue_size=50)
    rospy.Subscriber("/joint_states", JointState, js, queue_size=50)
    rospy.Subscriber("/cloud_registered", PointCloud2, counter("cloud"), queue_size=5)
    rospy.Subscriber("/obstacle_cloud", PointCloud2, counter("obs"), queue_size=5)
    rospy.Subscriber("/map", OccupancyGrid, counter("map"), queue_size=1)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    look_ok = look_n = 0
    ages_pelvis, ages_reloc = [], []
    t_end = time.time() + args.seconds
    while time.time() < t_end and not rospy.is_shutdown():
        look_n += 1
        try:
            t = buf.lookup_transform("map", "pelvis", rospy.Time(0))
            look_ok += 1
            ages_pelvis.append(rospy.Time.now().to_sec() - t.header.stamp.to_sec())
        except Exception:
            pass
        try:
            t = buf.lookup_transform("map", "camera_init", rospy.Time(0))
            ages_reloc.append(rospy.Time.now().to_sec() - t.header.stamp.to_sec())
        except Exception:
            pass
        time.sleep(0.05)

    S = args.seconds
    fails = []

    def check(name, cond, detail):
        print("[%s] %-46s %s" % ("PASS" if cond else "FAIL", name, detail))
        if not cond:
            fails.append(name)

    def mx(v):
        return max(map(abs, v)) if v else float("nan")

    check("/Odometry rate >= 5 Hz", cnt["odom"] / S >= 5, "%.1f Hz" % (cnt["odom"] / S))
    check("/joint_states rate >= 20 Hz", cnt["js"] / S >= 20, "%.1f Hz" % (cnt["js"] / S))
    check("/cloud_registered rate >= 5 Hz", cnt["cloud"] / S >= 5, "%.1f Hz" % (cnt["cloud"] / S))
    check("/obstacle_cloud rate >= 3 Hz", cnt["obs"] / S >= 3, "%.1f Hz" % (cnt["obs"] / S))
    check("/map received", cnt["map"] > 0, "%d msgs" % cnt["map"])
    check("|Odometry stamp - joint_states stamp| < timeout", bool(skew) and mx(skew) < args.timeout,
          "max %.3f s (limit %.2f)" % (mx(skew), args.timeout))
    check("Odometry stamped in ROS time (|now-stamp| < 1 s)", bool(odom_vs_now) and mx(odom_vs_now) < 1.0,
          "max |now - stamp| %.3f s" % mx(odom_vs_now))
    check("map->pelvis lookup succeeds >= 95 %", look_n and look_ok / look_n >= 0.95,
          "%d/%d" % (look_ok, look_n))
    check("map->pelvis newest transform < 0.5 s old", bool(ages_pelvis) and max(ages_pelvis) < 0.5,
          "max age %.3f s" % (max(ages_pelvis) if ages_pelvis else float("nan")))
    check("map->camera_init (relocalizer) < 0.5 s old", bool(ages_reloc) and max(ages_reloc) < 0.5,
          "max age %.3f s" % (max(ages_reloc) if ages_reloc else float("nan")))
    try:
        buf.lookup_transform("pelvis", "base_link", rospy.Time(0))
        print("[INFO] base_link is in the tree (with_base_link:=true): slam_health_check --component tf will FAIL")
    except Exception:
        print("[INFO] base_link not in the tree (default)")
    print("\nALL TF TIMING CHECKS PASSED" if not fails else "\nFAILED: " + "; ".join(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
