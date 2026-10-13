#!/usr/bin/env python3
"""Record where named objects ("table", "sofa") are on the REAL map.  Needs ROS only for `add`.

The navigation service  /navigation/navigate  looks the name up in  slam/maps/<map>/semantic.yaml.  Those numbers must
be MEASURED on the saved map - never typed from a guess.  Start the localization stack first so rviz shows the map:

    roslaunch system_bringup navigation.launch map_name:=lab joint_mode:=dds network_interface:=enp2s0
    rviz  (Fixed Frame = map, add the /map display, use the "Publish Point" tool)

    python3 tools/survey_semantic.py add table            # then click the middle of the table in rviz
    python3 tools/survey_semantic.py add sofa --approach-yaw-deg 180   # stand on the +x... side: see below
    python3 tools/survey_semantic.py add table --here --ahead 0.8      # robot localized, facing the object, 0.8 m away
    python3 tools/survey_semantic.py list
    python3 tools/survey_semantic.py remove table
    python3 tools/check_saved_map.py lab --require table,sofa          # then verify they can be reached

--approach-yaw-deg: direction FROM the object TOWARDS where the robot should stand, in the map frame
(0 = +x, 90 = +y).  Leave it out and the robot stands on the side it arrives from.
"""
import argparse
import math
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(REPO, "slam", "src"))

from g1_slam import grid_utils as gu  # noqa: E402
from g1_slam import semantic_store as ss  # noqa: E402


def map_dir(a):
    p = a.map if os.path.isdir(a.map) else os.path.join(a.maps_dir, a.map)
    if not os.path.isdir(p):
        sys.exit("map folder %s does not exist (map the site first: roslaunch g1_slam mapping.launch)" % p)
    return p


def inside_map(path, x, y):
    grid, res, ox, oy = gu.load_map_yaml(os.path.join(path, "map.yaml"))
    h, w = grid.shape
    return ox <= x < ox + w * res and oy <= y < oy + h * res


def point_from_click(timeout):
    import rospy
    import tf
    from geometry_msgs.msg import PointStamped
    rospy.init_node("survey_semantic", anonymous=True)
    print("Click the object in rviz with the 'Publish Point' tool (waiting %.0f s)..." % timeout)
    msg = rospy.wait_for_message("/clicked_point", PointStamped, timeout=timeout)
    if msg.header.frame_id != "map":
        lis = tf.TransformListener()
        lis.waitForTransform("map", msg.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        msg.header.stamp = rospy.Time(0)
        msg = lis.transformPoint("map", msg)
    return msg.point.x, msg.point.y


def point_from_robot(ahead):
    import rospy
    import tf
    rospy.init_node("survey_semantic", anonymous=True)
    lis = tf.TransformListener()
    lis.waitForTransform("map", "pelvis", rospy.Time(0), rospy.Duration(5.0))
    (x, y, _), q = lis.lookupTransform("map", "pelvis", rospy.Time(0))
    yaw = tf.transformations.euler_from_quaternion(q)[2]
    return x + ahead * math.cos(yaw), y + ahead * math.sin(yaw), yaw


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("add", "list", "remove"))
    ap.add_argument("name", nargs="?")
    ap.add_argument("--map", default="current")
    ap.add_argument("--maps-dir", default=os.path.join(REPO, "slam", "maps"))
    ap.add_argument("--here", action="store_true", help="use the robot's localized pose instead of an rviz click")
    ap.add_argument("--ahead", type=float, default=0.8, help="with --here: the object is this far in front of the robot")
    ap.add_argument("--approach-yaw-deg", type=float, default=None)
    ap.add_argument("--replace", action="store_true")
    ap.add_argument("--timeout", type=float, default=120.0)
    a = ap.parse_args(argv)
    path = map_dir(a)
    sem_file = os.path.join(path, "semantic.yaml")
    if a.action == "list":
        for name, w in sorted(ss.load(sem_file).items()):
            print("%-20s x=%7.3f y=%7.3f yaw=%6.3f approach_yaw=%s" % (name, w["x"], w["y"], w["yaw"], w["approach_yaw"]))
        return 0
    if not a.name:
        ap.error("%s needs an object name" % a.action)
    if a.action == "remove":
        ss.remove(sem_file, a.name)
        print("removed %s" % a.name)
        return 0
    yaw = 0.0
    if a.here:
        x, y, yaw = point_from_robot(a.ahead)
    else:
        x, y = point_from_click(a.timeout)
    if not inside_map(path, x, y):
        sys.exit("(%.2f, %.2f) is outside the saved map: not recorded. Wrong Fixed Frame, or not localized?" % (x, y))
    ay = None if a.approach_yaw_deg is None else math.radians(a.approach_yaw_deg)
    w = ss.add(sem_file, a.name, x, y, yaw, ay, replace=a.replace)
    print("recorded %s at map (%.3f, %.3f) in %s" % (ss.normalize_name(a.name), w["x"], w["y"], sem_file))
    print("next: python3 tools/check_saved_map.py %s --require %s" % (a.map, ss.normalize_name(a.name)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
