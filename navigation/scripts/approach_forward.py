#!/usr/bin/env python3
"""approach_forward - the last, guarded step to a table: walk STRAIGHT AHEAD a measured distance.

Why it exists: navigation keeps the pelvis >= 0.25 m from every obstacle (DANGER tier) and the live
obstacle layer ignores everything within 0.30 m of the pelvis (body_radius), so no navigation goal can
put the robot close enough to a table to grasp a bottle.  Run this AFTER navigation reported SUCCESS
at the standoff pose (e.g. goto_waypoint.py _name:=table_01):

    rosrun g1_navigation approach_forward.py _check:=true          # measure only, never moves
    rosrun g1_navigation approach_forward.py _distance:=0.20       # walk 0.20 m forward, then stop
    rosrun g1_navigation approach_forward.py _distance:=-0.20      # back out again after the grasp

How it moves: Twist on /navigation/cmd_vel, i.e. through cmd_vel_gate (needs
`rosservice call /navigation/enable_motion "data: true"`, clamps, 0.5 s watchdog) and locomotion_bridge
(its own clamps, watchdog and dead-band floor).  Heading is held to the start heading.  Distance is
measured in `camera_init` (FAST-LIO2 odometry: smooth, no relocalizer corrections mid-move).

Stops, whichever comes first:
  * travelled >= |distance| - stop_early                 -> SUCCESS
  * live front gap < min_front_gap (forward only)        -> STOPPED_GAP   (exit 3: at the obstacle, not an error)
  * no fresh /cloud_registered or TF for 0.5 s           -> ABORT        (exit 1)
  * timeout, motion gate closed, navigation busy, Ctrl+C -> ABORT        (exit 1)

Front gap: points of the live scan inside a corridor straight ahead of the body
(|y| <= corridor_half_width, x > body_front) and between min_height and max_height above the floor,
expressed in the pelvis frame; gap = nearest x - body_front.  Floor height comes from the saved map's
manifest (same rule as obstacle_cloud_node).  Run _check:=true first: if the arms are held forward
they can show up in the corridor; narrow `corridor_half_width` or raise `body_front` then.

Params: ~distance 0.20  ~speed 0.12  ~min_front_gap 0.03  ~stop_early 0.02  ~timeout 20  ~check false
        ~body_front 0.10  ~corridor_half_width 0.20  ~min_height 0.15  ~max_height 1.40
        ~yaw_gain 1.5  ~max_wz 0.3  ~odom_frame camera_init  ~map_frame map  ~robot_frame pelvis
        ~map_name current  ~maps_dir <g1_slam>/maps   (or ~floor_z_map directly)
"""
import math
import os
import sys

import numpy as np

SUCCESS, STOPPED_GAP, ABORT = "SUCCESS", "STOPPED_GAP", "ABORT"


def front_gap(pts_robot, z_map, floor_z, body_front=0.10, half_width=0.20, min_h=0.15, max_h=1.40):
    """pts_robot: Nx3 in the robot frame (x forward, y left).  z_map: N heights in the z-up map frame.
    -> (gap metres or inf, number of corridor points)."""
    if pts_robot.size == 0:
        return float("inf"), 0
    h = z_map - floor_z
    m = ((pts_robot[:, 0] > body_front) & (np.abs(pts_robot[:, 1]) <= half_width)
         & (h >= min_h) & (h <= max_h))
    if not m.any():
        return float("inf"), 0
    return float(pts_robot[m, 0].min() - body_front), int(m.sum())


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def step(start, pose, distance, gap, cfg):
    """One control decision.  start/pose = (x, y, yaw) in odom.  -> (status or None, vx, wz, travelled)."""
    x0, y0, yaw0 = start
    travelled = (pose[0] - x0) * math.cos(yaw0) + (pose[1] - y0) * math.sin(yaw0)   # signed, along start heading
    sgn = 1.0 if distance >= 0 else -1.0
    if sgn * travelled >= abs(distance) - cfg["stop_early"]:
        return SUCCESS, 0.0, 0.0, travelled
    if sgn > 0 and gap < cfg["min_front_gap"]:
        return STOPPED_GAP, 0.0, 0.0, travelled
    wz = max(-cfg["max_wz"], min(cfg["max_wz"], -cfg["yaw_gain"] * wrap(pose[2] - yaw0)))
    return None, sgn * cfg["speed"], wz, travelled


def _T(tf):
    from g1_slam import cloud_utils as cu
    t, q = tf.transform.translation, tf.transform.rotation
    return cu.make_T(cu.quat_to_R(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])


def main():
    import rospkg
    import rospy
    import tf2_ros
    import yaml
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String
    from g1_slam import cloud_utils as cu

    rospy.init_node("approach_forward")
    p = lambda n, d: rospy.get_param("~" + n, d)  # noqa: E731
    cfg = {"speed": abs(float(p("speed", 0.12))), "min_front_gap": float(p("min_front_gap", 0.03)),
           "stop_early": float(p("stop_early", 0.02)), "yaw_gain": float(p("yaw_gain", 1.5)),
           "max_wz": float(p("max_wz", 0.3))}
    distance = float(p("distance", 0.20))
    check = bool(p("check", False))
    timeout = float(p("timeout", 20.0))
    geo = dict(body_front=float(p("body_front", 0.10)), half_width=float(p("corridor_half_width", 0.20)),
               min_h=float(p("min_height", 0.15)), max_h=float(p("max_height", 1.40)))
    odom_f, map_f, rob_f = p("odom_frame", "camera_init"), p("map_frame", "map"), p("robot_frame", "pelvis")
    cloud_frame = p("cloud_frame", "camera_init")
    floor = p("floor_z_map", None)
    map_name, maps_dir_p = str(p("map_name", "current")), str(p("maps_dir", ""))
    cloud_topic, cmd_topic = p("cloud_topic", "/cloud_registered"), p("cmd_topic", "/navigation/cmd_vel")
    # rosrun's _name:=value lands on the parameter server and would SILENTLY carry into the next run
    # (a stale _check:=true makes a real run a no-op; a stale _distance would move the wrong amount).
    # Everything is read above, so clear this node's private namespace now.
    ns = rospy.get_name() + "/"
    for name in rospy.get_param_names():
        if name.startswith(ns):
            try:
                rospy.delete_param(name)
            except KeyError:
                pass

    if floor is None:
        maps_dir = os.path.expanduser(maps_dir_p or os.path.join(rospkg.RosPack().get_path("g1_slam"), "maps"))
        man = os.path.join(maps_dir, map_name, "manifest.yaml")
        try:
            floor = -float(yaml.safe_load(open(man))["floor_z_camera_init"])
        except Exception as exc:  # noqa: BLE001
            rospy.logfatal("[approach_forward] no floor height (%s: %s); pass _floor_z_map:=", man, exc)
            return 2
    floor = float(floor)

    buf = tf2_ros.Buffer(rospy.Duration(5.0))
    tf2_ros.TransformListener(buf)
    st = {"gap": float("inf"), "n": 0, "t": 0.0, "gate": None, "nav": ""}

    def cloud_cb(msg):
        try:
            T_r = _T(buf.lookup_transform(rob_f, cloud_frame, rospy.Time(0), rospy.Duration(0.1)))
            T_m = _T(buf.lookup_transform(map_f, cloud_frame, rospy.Time(0), rospy.Duration(0.1)))
        except Exception:  # noqa: BLE001
            return
        pts = cu.pc2_to_xyz(msg)
        if pts.size == 0:
            return
        st["gap"], st["n"] = front_gap(cu.apply_T(T_r, pts), cu.apply_T(T_m, pts)[:, 2], floor, **geo)
        st["t"] = rospy.Time.now().to_sec()

    rospy.Subscriber(cloud_topic, PointCloud2, cloud_cb, queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber("/navigation/motion_enabled", Bool, lambda m: st.update(gate=bool(m.data)), queue_size=1)
    rospy.Subscriber("/navigation/status", String, lambda m: st.update(nav=m.data), queue_size=1)
    pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)

    def pose():
        tf = buf.lookup_transform(odom_f, rob_f, rospy.Time(0), rospy.Duration(0.2))
        T = _T(tf)
        return T[0, 3], T[1, 3], cu.yaw_of(T[:3, :3])

    deadline = rospy.Time.now().to_sec() + 5.0
    while not rospy.is_shutdown() and st["t"] == 0.0 and rospy.Time.now().to_sec() < deadline:
        rospy.sleep(0.05)
    if st["t"] == 0.0:
        rospy.logerr("[approach_forward] ABORT: no /cloud_registered + TF %s<-%s within 5 s", rob_f, cloud_frame)
        return 1
    print("front gap %.3f m (%d corridor points, floor z %.2f in %s)" % (st["gap"], st["n"], floor, map_f))
    if check:
        print("check only: nothing was sent")
        return 0
    if distance > 0 and distance > st["gap"] - cfg["min_front_gap"] + cfg["stop_early"]:
        print("note: the live gap allows only %.3f m; the gap guard will stop the robot first"
              % max(0.0, st["gap"] - cfg["min_front_gap"]))
    rospy.sleep(0.3)
    if st["gate"] is not True:
        rospy.logerr("[approach_forward] ABORT: /navigation/motion_enabled is not True. Open it deliberately: "
                     "rosservice call /navigation/enable_motion \"data: true\"")
        return 1
    if st["nav"] in ("PLANNING", "FOLLOWING", "RECOVERING"):
        rospy.logerr("[approach_forward] ABORT: navigation is %s; wait for it to finish or cancel it", st["nav"])
        return 1

    stop = Twist()
    rospy.on_shutdown(lambda: pub.publish(stop))
    try:
        start = pose()
    except Exception as exc:  # noqa: BLE001
        rospy.logerr("[approach_forward] ABORT: no %s->%s (%s)", odom_f, rob_f, exc)
        return 1
    t_end = rospy.Time.now().to_sec() + timeout
    rate = rospy.Rate(20.0)
    status, why, travelled = ABORT, "interrupted", 0.0
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        if now > t_end:
            why = "timeout %.0f s" % timeout
            break
        if st["gate"] is not True:
            why = "motion gate closed"
            break
        if now - st["t"] > 0.5:
            why = "live scan stale"
            break
        try:
            cur = pose()
        except Exception:  # noqa: BLE001
            why = "TF lost"
            break
        res, vx, wz, travelled = step(start, cur, distance, st["gap"], cfg)
        if res is not None:
            status, why = res, ""
            break
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = vx, wz
        pub.publish(cmd)
        rate.sleep()
    for _ in range(5):
        pub.publish(stop)
        rospy.sleep(0.05)
    rospy.sleep(1.0)                                   # let the last step finish before measuring
    try:
        cur = pose()
        travelled = (cur[0] - start[0]) * math.cos(start[2]) + (cur[1] - start[1]) * math.sin(start[2])
    except Exception:  # noqa: BLE001
        pass
    print("%s: travelled %+.3f m of %+.3f m, front gap now %.3f m%s"
          % (status, travelled, distance, st["gap"], (" (" + why + ")") if why else ""))
    return {SUCCESS: 0, STOPPED_GAP: 3}.get(status, 1)


if __name__ == "__main__":
    sys.exit(main())
