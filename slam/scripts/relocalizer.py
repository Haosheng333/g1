#!/usr/bin/env python3
"""relocalizer - finds the robot in a SAVED map and broadcasts  map -> camera_init.

Why it exists: FAST-LIO2 re-creates `camera_init` at the power-on pose of every
run and has no prior-map loading (slam/README.md section 7).  This node matches
the live /cloud_registered (expressed in the NEW camera_init) against the saved
`map.pcd` (expressed in the mapping session's camera_init) with a coarse global
search + point-to-point ICP, and publishes

        map  ->  camera_init          (dynamic TF, 10 Hz, only once localized)

where `map` is the z-UP navigation frame:
        T_map_camera_init = T_map_saved @ T_saved_live
        T_map_saved       = rpy(map_to_saved_rpy)   (default roll pi)

Safety rules before declaring "localized":
  * fitness >= min_fitness and inlier RMSE <= max_rmse
  * the best pose beats the best DISTINCT alternative by >= ambiguity_margin
    (symmetric rooms -> refuse and ask for an rviz "2D Pose Estimate")
  * the same pose is found `confirm_count` attempts in a row

Topics
  sub  /cloud_registered              PointCloud2 (live scan in camera_init)
  sub  /initialpose                   PoseWithCovarianceStamped (frame map), optional seed
  sub  /slam/relocalizer/reset        Empty, start searching again
  pub  /slam/relocalizer/localized    Bool (latched)
  pub  /slam/relocalizer/fitness      Float32
  pub  /slam/relocalizer/status       String
  TF   map -> camera_init
"""
import collections
import math
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty, Float32, String

from g1_slam import cloud_utils as cu

SEARCHING, LOCALIZED = "SEARCHING", "LOCALIZED"


class Relocalizer:
    def __init__(self):
        rospy.init_node("relocalizer")
        p = rospy.get_param
        pcd_path = p("~map_pcd")
        self.map_frame = p("~map_frame", "map")
        self.odom_frame = p("~odom_frame", "camera_init")
        self.robot_frame = p("~robot_frame", "pelvis")
        self.match_voxel = float(p("~match_voxel", 0.25))
        self.scan_voxel = float(p("~scan_voxel", 0.15))
        self.n_scans = int(p("~accumulate_scans", 40))
        self.min_scans = int(p("~min_scans", 15))
        self.min_points = int(p("~min_points", 1500))
        self.period = float(p("~attempt_period", 2.0))
        self.yaw_step = float(p("~yaw_step_deg", 15.0))
        self.trans_range = float(p("~trans_range", 4.0))
        self.trans_step = float(p("~trans_step", 1.0))
        self.top_k = int(p("~top_k", 8))
        self.min_fitness = float(p("~min_fitness", 0.60))
        self.max_rmse = float(p("~max_rmse", 0.20))
        self.final_inlier = float(p("~final_inlier_dist", 0.30))
        self.margin = float(p("~ambiguity_margin", 0.03))
        self.confirm_count = int(p("~confirm_count", 2))
        self.confirm_pos = float(p("~confirm_pos", 0.25))
        self.confirm_yaw = float(p("~confirm_yaw", 0.15))
        self.tf_rate = float(p("~tf_rate", 10.0))
        rpy = list(p("~map_to_saved_rpy", [math.pi, 0.0, 0.0]))
        self.T_map_saved = cu.make_T(cu.rpy_to_R(*rpy), [0.0, 0.0, 0.0])

        rospy.loginfo("[relocalizer] loading %s", pcd_path)
        pts = cu.read_pcd_xyz(pcd_path)
        self.dst = cu.voxel_downsample(pts, self.match_voxel)
        if len(self.dst) < 500:
            rospy.logfatal("[relocalizer] map has only %d points after downsampling", len(self.dst))
            raise rospy.ROSInitException("map too small")
        self.tree = cKDTree(self.dst)
        rospy.loginfo("[relocalizer] map: %d raw -> %d matching points", len(pts), len(self.dst))

        self.lock = threading.Lock()
        self.buffer = collections.deque(maxlen=self.n_scans)
        self.state = SEARCHING
        self.busy = False
        self.seed = None            # (x, y, yaw) of the robot in `map`, from rviz
        self.last_accept = None     # (T_saved_live, count)
        self.T_map_odom = None

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.loc_pub = rospy.Publisher("/slam/relocalizer/localized", Bool, queue_size=1, latch=True)
        self.fit_pub = rospy.Publisher("/slam/relocalizer/fitness", Float32, queue_size=1)
        self.status_pub = rospy.Publisher("/slam/relocalizer/status", String, queue_size=1, latch=True)
        self.loc_pub.publish(Bool(data=False))

        rospy.Subscriber("/cloud_registered", PointCloud2, self.cloud_cb, queue_size=3)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.seed_cb, queue_size=1)
        rospy.Subscriber("/slam/relocalizer/reset", Empty, self.reset_cb, queue_size=1)
        rospy.Timer(rospy.Duration(self.period), self.attempt)
        rospy.Timer(rospy.Duration(1.0 / self.tf_rate), self.publish_tf)
        self.status("SEARCHING: collecting scans")

    # ------------------------------------------------------------------ io
    def status(self, text):
        self.status_pub.publish(String(data=text))
        rospy.loginfo("[relocalizer] %s", text)

    def cloud_cb(self, msg):
        if self.state != SEARCHING:
            return
        try:
            pts = cu.voxel_downsample(cu.pc2_to_xyz(msg), self.scan_voxel)
        except ValueError as exc:
            rospy.logerr_throttle(5.0, "[relocalizer] bad cloud: %s", exc)
            return
        with self.lock:
            self.buffer.append(pts)

    def seed_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.seed = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        rospy.loginfo("[relocalizer] seed pose (map): x=%.2f y=%.2f yaw=%.2f", *self.seed)

    def reset_cb(self, _msg):
        with self.lock:
            self.buffer.clear()
        self.state = SEARCHING
        self.last_accept = None
        self.T_map_odom = None
        self.loc_pub.publish(Bool(data=False))
        self.status("SEARCHING: reset requested")

    # -------------------------------------------------------------- seeding
    def seed_transform(self):
        """T_saved_live from the rviz 2D pose estimate + live TF camera_init->pelvis."""
        if self.seed is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.robot_frame, rospy.Time(0),
                                                 rospy.Duration(0.5))
        except Exception as exc:  # noqa: BLE001 - tf2 raises several exception types
            rospy.logwarn("[relocalizer] cannot use seed, no TF %s->%s: %s",
                          self.odom_frame, self.robot_frame, exc)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        T_odom_robot = cu.make_T(cu.quat_to_R(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])
        x, y, yaw = self.seed
        # map is z-up and the pelvis is upright; its height in `map` is -z of camera_init.
        T_map_robot = cu.make_T(cu.rot_z(yaw), [x, y, -t.z])
        T_map_odom = T_map_robot @ cu.invert_T(T_odom_robot)
        return cu.invert_T(self.T_map_saved) @ T_map_odom

    # -------------------------------------------------------------- attempt
    def attempt(self, _evt):
        if self.state != SEARCHING or self.busy:
            return
        self.busy = True
        try:
            with self.lock:
                scans = list(self.buffer)
            if len(scans) < self.min_scans:
                self.status("SEARCHING: %d/%d scans" % (len(scans), self.min_scans))
                return
            src = cu.voxel_downsample(np.vstack(scans), self.match_voxel)
            if len(src) < self.min_points:
                self.status("SEARCHING: %d/%d points" % (len(src), self.min_points))
                return
            seed_T = self.seed_transform()
            T, fit, rmse, runner_up = cu.register(
                src, self.dst, self.tree,
                init_Ts=[seed_T] if seed_T is not None else None,
                final_inlier_dist=self.final_inlier,
                yaw_step_deg=self.yaw_step, trans_range=self.trans_range,
                trans_step=self.trans_step, top_k=self.top_k)
            self.fit_pub.publish(Float32(data=fit))

            if fit < self.min_fitness or rmse > self.max_rmse:
                self.last_accept = None
                self.status("SEARCHING: best fitness %.2f rmse %.3f (need >=%.2f, <=%.2f)"
                            % (fit, rmse, self.min_fitness, self.max_rmse))
                return
            if seed_T is None and fit - runner_up < self.margin:
                self.last_accept = None
                self.status("SEARCHING: AMBIGUOUS match (best %.2f, alternative %.2f). Give an rviz "
                            "'2D Pose Estimate' (topic /initialpose) to disambiguate." % (fit, runner_up))
                return

            if self.last_accept is not None:
                prev, count = self.last_accept
                e = cu.invert_T(prev) @ T
                if np.linalg.norm(e[:3, 3]) <= self.confirm_pos and abs(cu.yaw_of(e[:3, :3])) <= self.confirm_yaw:
                    count += 1
                else:
                    count = 1
            else:
                count = 1
            self.last_accept = (T, count)
            if count < self.confirm_count:
                self.status("SEARCHING: candidate fitness %.2f rmse %.3f, confirming %d/%d"
                            % (fit, rmse, count, self.confirm_count))
                return

            self.T_map_odom = self.T_map_saved @ T
            self.state = LOCALIZED
            self.loc_pub.publish(Bool(data=True))
            yaw = cu.yaw_of(self.T_map_odom[:3, :3])
            self.status("LOCALIZED: fitness %.2f rmse %.3f, map->%s at (%.2f, %.2f) yaw_rot %.2f rad"
                        % (fit, rmse, self.odom_frame, self.T_map_odom[0, 3], self.T_map_odom[1, 3], yaw))
        finally:
            self.busy = False

    # ------------------------------------------------------------------- tf
    def publish_tf(self, _evt):
        T = self.T_map_odom
        if self.state != LOCALIZED or T is None:
            return
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = T[:3, 3]
        qx, qy, qz, qw = cu.R_to_quat(T[:3, :3])
        msg.transform.rotation.x, msg.transform.rotation.y = qx, qy
        msg.transform.rotation.z, msg.transform.rotation.w = qz, qw
        self.br.sendTransform(msg)


if __name__ == "__main__":
    try:
        Relocalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
