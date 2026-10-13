#!/usr/bin/env python3
"""slam_manager - owns the SLAM session lifecycle (doc section 4.1 / 4.2).

mapping mode      accumulates FAST-LIO2's /cloud_registered (voxel-merged) and
                  offers   /slam/manager/save_current_map  (std_srvs/Trigger)
                  which writes, into  <maps_dir>/<name>/ :
                      map.pcd        canonical cloud, camera_init frame (z-DOWN);
                                     this is what the relocalizer matches against
                      map_zup.pcd    same cloud rotated into the z-UP `map` frame
                      map.pgm/.yaml  2D occupancy grid (via g1_slam pcd_to_map)
                      manifest.yaml  metadata
                      semantic.yaml  named objects (empty template, filled by hand)
                  and re-points the symlink  <maps_dir>/current -> <name>.

localization mode mirrors the relocalizer's result onto the latched flag
                  /slam/manager/state (std_msgs/Bool, True = localized) that the
                  navigation node gates on, and offers
                  /slam/manager/relocalize  (std_srvs/Trigger) to start over.

Topics
  pub  /slam/manager/state    std_msgs/Bool    latched, True = localized to a saved map
  pub  /slam/manager/status   std_msgs/String  latched, human readable
  sub  /cloud_registered      (mapping mode)
  sub  /slam/relocalizer/localized (localization mode)
"""
import math
import os
import subprocess
import threading
import time

import numpy as np
import rospkg
import rospy
import yaml
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger, TriggerResponse

from g1_slam import cloud_utils as cu
from g1_slam import map_store as ms

def _fnum(value):
    """'auto'/''/None -> None, otherwise float."""
    if value is None:
        return None
    s = str(value).strip().lower()
    if s in ("", "auto", "nan", "none"):
        return None
    return float(s)


class SlamManager:
    def __init__(self):
        rospy.init_node("slam_manager")
        self.mode = rospy.get_param("~mode", "mapping")
        if self.mode not in ("mapping", "localization"):
            raise rospy.ROSInitException("~mode must be mapping|localization")
        default_dir = os.path.join(rospkg.RosPack().get_path("g1_slam"), "maps")
        self.maps_dir = os.path.expanduser(rospy.get_param("~maps_dir", default_dir))
        self.map_name = str(rospy.get_param("~map_name", "")).strip()
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.voxel = float(rospy.get_param("~voxel", 0.05))   # == grid resolution, or floor cells stay empty
        self.max_points = int(rospy.get_param("~max_points", 8000000))
        self.min_points = int(rospy.get_param("~min_points_to_save", 5000))
        self.floor_z = _fnum(rospy.get_param("~floor_z", "auto"))
        self.obs_min_h = float(rospy.get_param("~obstacle_min_height", 0.15))
        self.obs_max_h = float(rospy.get_param("~obstacle_max_height", 1.80))
        self.resolution = float(rospy.get_param("~grid_resolution", 0.05))
        self.map_to_saved_rpy = list(rospy.get_param("~map_to_saved_rpy", [math.pi, 0.0, 0.0]))

        self.state_pub = rospy.Publisher("/slam/manager/state", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/slam/manager/status", String, queue_size=1, latch=True)
        self.state_pub.publish(Bool(data=False))

        self.lock = threading.Lock()
        rospy.Service("/slam/manager/save_current_map", Trigger, self.save_cb)

        if self.mode == "mapping":
            self.cloud = np.zeros((0, 3), dtype=np.float32)
            self.pending = []
            self.scans = 0
            self.full_warned = False
            rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=5)
            rospy.Timer(rospy.Duration(2.0), self.merge_timer)
            self.set_status("MAPPING: waiting for %s" % self.cloud_topic)
        else:
            self.reset_pub = rospy.Publisher("/slam/relocalizer/reset", Empty, queue_size=1)
            rospy.Subscriber("/slam/relocalizer/localized", Bool, self.localized_cb, queue_size=1)
            rospy.Service("/slam/manager/relocalize", Trigger, self.relocalize_cb)
            self.set_status(self._localization_start_status())
        rospy.loginfo("[slam_manager] mode=%s maps_dir=%s", self.mode, self.maps_dir)

    # ------------------------------------------------------------------ misc
    def set_status(self, text):
        self.status_pub.publish(String(data=text))
        rospy.loginfo("[slam_manager] %s", text)

    # --------------------------------------------------------- mapping mode
    def cloud_cb(self, msg):
        try:
            pts = cu.voxel_downsample(cu.pc2_to_xyz(msg), self.voxel)
        except ValueError as exc:
            rospy.logerr_throttle(5.0, "[slam_manager] bad cloud: %s", exc)
            return
        with self.lock:
            self.pending.append(pts)
            self.scans += 1

    def _merge(self):
        with self.lock:
            pending, self.pending = self.pending, []
        if not pending:
            return
        if len(self.cloud) >= self.max_points:
            if not self.full_warned:
                rospy.logwarn("[slam_manager] map reached max_points=%d; new points ignored. "
                              "Save the map now.", self.max_points)
                self.full_warned = True
            return
        merged = cu.voxel_downsample(np.vstack([self.cloud] + pending), self.voxel)
        with self.lock:
            self.cloud = merged

    def merge_timer(self, _evt):
        self._merge()
        self.status_pub.publish(String(
            data="MAPPING: %d scans, %d map points (voxel %.2f m)" % (self.scans, len(self.cloud), self.voxel)))

    # ------------------------------------------------------------ save map
    def save_cb(self, _req):
        if self.mode != "mapping":
            return TriggerResponse(success=False,
                                   message="save_current_map only works in mapping mode")
        try:
            return TriggerResponse(success=True, message=self._save())
        except Exception as exc:  # noqa: BLE001 - report every failure to the operator
            rospy.logerr("[slam_manager] save failed: %s", exc)
            self.set_status("MAPPING: save FAILED: %s" % exc)
            return TriggerResponse(success=False, message="save failed: %s" % exc)

    def _grid_runner(self, pcd_zup, out_dir, floor_up):
        cmd = ["rosrun", "g1_slam", "pcd_to_map",
               "--input", pcd_zup, "--name", "map", "--output-dir", out_dir,
               "--resolution", str(self.resolution),
               "--z-min", "%.3f" % (floor_up + self.obs_min_h),
               "--z-max", "%.3f" % (floor_up + self.obs_max_h),
               "--ground-z-min", "%.3f" % (floor_up - 0.30)]
        rospy.loginfo("[slam_manager] running: %s", " ".join(cmd))
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        if proc.returncode != 0:
            raise ms.MapError("pcd_to_map failed (exit %d): %s" % (proc.returncode, proc.stderr.strip()[-400:]))

    def _save(self):
        self._merge()
        with self.lock:
            cloud = self.cloud.copy()
        if len(cloud) < self.min_points:
            raise RuntimeError("only %d map points (< %d). Drive the robot around first."
                               % (len(cloud), self.min_points))

        floor_down = self.floor_z if self.floor_z is not None else cu.estimate_floor_z_down(cloud)
        if floor_down is None or not (0.2 < floor_down < 3.0):
            raise RuntimeError("could not find the floor (estimate=%s). camera_init is z-DOWN so the "
                               "floor must be at z = +IMU height (0.2..3 m). Pass floor_z:=<value>."
                               % floor_down)

        name = ms.unique_name(self.maps_dir, self.map_name)
        # staging dir -> verify -> rename -> re-point `current`; any failure leaves nothing behind
        out, info = ms.save_map(self.maps_dir, name, cloud, floor_down, self._grid_runner, self.voxel,
                                self.resolution, self.map_to_saved_rpy, (self.obs_min_h, self.obs_max_h))
        msg = ("saved + verified map '%s' (%d points, %.1f m^2 free, floor z=%.2f) in %s ; 'current' now points "
               "to it. NEXT: python3 tools/check_saved_map.py %s ; then survey objects: "
               "python3 tools/survey_semantic.py add table"
               % (name, info["num_points"], info["free_area_m2"], floor_down, out, name))
        self.set_status("MAPPING: " + msg)
        return msg

    # ------------------------------------------------- localization mode
    def _localization_start_status(self):
        """Check the saved map BEFORE the robot depends on it; the flag stays False if it is unusable."""
        path = os.path.join(self.maps_dir, self.map_name or "current")
        problems, info = ms.verify_map_dir(path)
        if problems:
            rospy.logerr("[slam_manager] map '%s' is NOT usable: %s", self.map_name, "; ".join(problems))
            return "LOCALIZATION: map '%s' unusable: %s" % (self.map_name, "; ".join(problems))
        if not info.get("semantic"):
            rospy.logwarn("[slam_manager] %s/semantic.yaml has no waypoints: /navigation/navigate will reject "
                          "every object name. Survey them: python3 tools/survey_semantic.py add <name>", path)
        return "LOCALIZATION: map '%s' verified (%d points), not localized yet" % (self.map_name, info.get("num_points", 0))

    def localized_cb(self, msg):
        self.state_pub.publish(Bool(data=bool(msg.data)))
        self.set_status("LOCALIZED to '%s'" % self.map_name if msg.data
                        else "LOCALIZATION: map '%s', searching" % self.map_name)

    def relocalize_cb(self, _req):
        self.state_pub.publish(Bool(data=False))
        self.reset_pub.publish(Empty())
        self.set_status("LOCALIZATION: relocalization requested")
        return TriggerResponse(success=True, message="relocalizer reset")


if __name__ == "__main__":
    try:
        SlamManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
