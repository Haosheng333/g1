#!/usr/bin/env python3
"""obstacle_cloud_node - produces /obstacle_cloud, the live lethal layer of doc section 4.3.

    /cloud_registered (FAST-LIO2 scan, frame camera_init, z DOWN)
        --TF map<-camera_init (relocalizer)-->  z-UP map frame
        --keep points between min_height and max_height above the floor-->
        --drop points on the robot's own body-->  /obstacle_cloud  (PointCloud2, frame map, ~10 Hz)

The floor height in the z-up map frame is  -floor_z_camera_init  from the saved map's manifest.yaml
(written by slam_manager).  There is NO default: guessing a floor height would silently put the floor
into the obstacle layer, so the node refuses to start without it.

Params
  ~manifest            path to <map>/manifest.yaml        (or ~floor_z_map directly)
  ~floor_z_map         floor z in the map frame [m] (overrides manifest)
  ~cloud_topic         /cloud_registered
  ~cloud_frame         camera_init
  ~map_frame           map
  ~robot_frame         pelvis
  ~min_height          0.15   points lower than this above the floor are floor
  ~max_height          1.60   points higher than this are overhead (ceiling, lamps)
  ~body_radius         0.30   horizontal radius around the pelvis ignored (robot's own arms / torso)
  ~voxel               0.05
"""
import sys

import numpy as np
import rospy
import tf2_ros
import yaml
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from g1_slam import cloud_utils as cu


def main():
    rospy.init_node("obstacle_cloud_node")
    p = rospy.get_param
    cloud_topic = p("~cloud_topic", "/cloud_registered")
    cloud_frame = p("~cloud_frame", "camera_init")
    map_frame = p("~map_frame", "map")
    robot_frame = p("~robot_frame", "pelvis")
    min_h = float(p("~min_height", 0.15))
    max_h = float(p("~max_height", 1.60))
    body_r = float(p("~body_radius", 0.30))
    voxel = float(p("~voxel", 0.05))

    floor = p("~floor_z_map", None)
    if floor is None:
        manifest = p("~manifest", "")
        try:
            with open(manifest) as f:
                floor = -float(yaml.safe_load(f)["floor_z_camera_init"])   # roll pi flips z
        except Exception as exc:  # noqa: BLE001
            rospy.logfatal("[obstacle_cloud] no floor height: set ~floor_z_map or a valid ~manifest "
                           "(got %r: %s)", manifest, exc)
            sys.exit(2)
    floor = float(floor)
    rospy.loginfo("[obstacle_cloud] floor z in %s = %.3f m; keeping %.2f..%.2f m above it",
                  map_frame, floor, min_h, max_h)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    pub = rospy.Publisher("/obstacle_cloud", PointCloud2, queue_size=1)

    def tf_matrix(target, source, stamp):
        t = buf.lookup_transform(target, source, stamp, rospy.Duration(0.1))
        q, tr = t.transform.rotation, t.transform.translation
        return cu.make_T(cu.quat_to_R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])

    def cb(msg):
        try:
            pts = cu.pc2_to_xyz(msg)
            if len(pts) == 0:
                return
            T = tf_matrix(map_frame, msg.header.frame_id or cloud_frame, rospy.Time(0))
            pts = cu.apply_T(T, pts)
            keep = (pts[:, 2] >= floor + min_h) & (pts[:, 2] <= floor + max_h)
            pts = pts[keep]
            try:
                b = tf_matrix(map_frame, robot_frame, rospy.Time(0))[:3, 3]
                pts = pts[np.hypot(pts[:, 0] - b[0], pts[:, 1] - b[1]) > body_r]
            except tf2_ros.TransformException:
                pass     # no robot TF yet: keep everything (safer than dropping obstacles)
            pts = cu.voxel_downsample(pts, voxel)
            header = Header(stamp=msg.header.stamp, frame_id=map_frame)
            pub.publish(point_cloud2.create_cloud_xyz32(header, pts.tolist()))
        except tf2_ros.TransformException as exc:
            rospy.logwarn_throttle(5.0, "[obstacle_cloud] waiting for TF %s<-%s: %s", map_frame, cloud_frame, exc)
        except ValueError as exc:
            rospy.logerr_throttle(5.0, "[obstacle_cloud] bad cloud: %s", exc)

    rospy.Subscriber(cloud_topic, PointCloud2, cb, queue_size=1, buff_size=2 ** 24)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
