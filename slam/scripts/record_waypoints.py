#!/usr/bin/env python3
"""Record the robot's own STANDING pose, one key press at a time.

    rosrun g1_slam record_waypoints.py
    rosrun g1_slam record_waypoints.py _prefix:=handover _snapshot:=true

Teleoperate the robot to the spot, stop, adjust its heading, then:

    A   save this standing pose under a new unique name
    L   list what has been recorded so far
    Q   quit

THIS NODE NEVER MOVES THE ROBOT. It builds no goal publisher, no cmd_vel
publisher and no motion-gate client -- grep the source: there is nothing here
that could command motion. `A` writes a YAML record and nothing else. Driving is
your teleop; navigating back is the separate `goto_waypoint.py`.

KEYBOARD, WITHOUT ROOT. Keys are read from this terminal's own stdin in cbreak
mode via `termios`/`tty`. No `keyboard` module, no `/dev/input/event*`, no
`sudo`, and nothing is captured from other windows -- the terminal must have
focus. If stdin is not a TTY (a pipe, a launch file) the node says so and exits
rather than silently recording nothing.

WHAT IS SAVED, and the checks before it is
Each record carries x, y and yaw in the z-UP `map` frame, the ROS stamp of the
transform it came from, the frame names, and the identity of the saved map. A
press is REFUSED, with the reason printed, unless all of these hold:

  * `/slam/manager/state` is True -- the robot is localized to a saved map.
    Without it `map -> camera_init` is not published at all, so there are no
    map coordinates to record.
  * the `map -> <robot_frame>` transform exists and is FRESH (`~max_tf_age`).
  * the transform's own frames match the configured ones, so an odometry-frame
    pose can never be written into a map-frame file.
  * the file's map identity matches the live one. A mismatch is refused, never
    appended: two maps' coordinate origins must not share a file.

Odometry coordinates are therefore never used silently: either the map chain is
up and localized, or nothing is written.
"""
import datetime
import os
import sys
import termios
import time
import tty

import rospy
import tf2_ros
import yaml
from std_msgs.msg import Bool

from g1_slam import cloud_utils as cu
from g1_slam import standing_store as store

HELP = "  [A] save standing pose   [L] list   [Q] quit"


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _yaw_from_quaternion(x, y, z, w):
    import math
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Recorder(object):
    def __init__(self):
        rospy.init_node("record_waypoints")

        self.maps_dir = os.path.expanduser(str(_param("maps_dir", "")))
        if not self.maps_dir:
            from rospkg import RosPack
            self.maps_dir = os.path.join(RosPack().get_path("g1_slam"), "maps")
        self.map_name = str(_param("map_name", "current")).strip() or "current"
        self.map_frame = str(_param("map_frame", "map"))
        self.robot_frame = str(_param("robot_frame", "pelvis"))
        self.prefix = str(_param("prefix", "handover"))
        self.max_tf_age = float(_param("max_tf_age", 0.5))
        self.tf_timeout = float(_param("tf_timeout", 1.0))
        self.require_localized = bool(_param("require_localized", True))
        self.snapshot = bool(_param("snapshot", False))
        self.snapshot_radius = float(_param("snapshot_radius", 3.0))
        self.snapshot_topic = str(_param("snapshot_topic", "/cloud_registered"))

        self.map_dir = os.path.join(self.maps_dir, self.map_name)
        self.waypoints_file = os.path.expanduser(str(_param(
            "waypoints_file", os.path.join(self.map_dir, "standing_waypoints.yaml"))))

        self.identity = self._read_map_identity()

        # Localisation readiness. Latched, so one subscribe is enough.
        self._localized = None
        rospy.Subscriber("/slam/manager/state", Bool, self._on_state, queue_size=1)

        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(float(_param("tf_cache", 10.0))))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Optional environmental reference. Subscribed ONLY when asked for.
        self._cloud = None
        self._cloud_at = None
        if self.snapshot:
            from sensor_msgs.msg import PointCloud2
            rospy.Subscriber(self.snapshot_topic, PointCloud2, self._on_cloud,
                             queue_size=1, buff_size=2 ** 24)

        rospy.loginfo("[record_waypoints] map '%s' (created %s)",
                      self.identity["name"], self.identity["created_at"])
        rospy.loginfo("[record_waypoints] file: %s", self.waypoints_file)
        rospy.loginfo("[record_waypoints] THIS NODE NEVER MOVES THE ROBOT")

    # ------------------------------------------------------------------ inputs
    def _read_map_identity(self):
        """Map identity from the map's own manifest.yaml. No manifest, no recording."""
        manifest = os.path.join(self.map_dir, "manifest.yaml")
        if not os.path.isfile(manifest):
            raise rospy.ROSInitException(
                "no manifest at %s: cannot establish which map these coordinates would "
                "belong to. Point _map_name:= at a saved map (or _maps_dir:=)." % manifest)
        with open(manifest, encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        name = str(data.get("map_name", "")).strip()
        created = str(data.get("created_at", "")).strip()
        if not name:
            raise rospy.ROSInitException("%s has no map_name" % manifest)
        frame = str(data.get("map_frame", self.map_frame)) or self.map_frame
        if frame != self.map_frame:
            rospy.logwarn("[record_waypoints] manifest map_frame %r differs from "
                          "~map_frame %r; using the manifest's", frame, self.map_frame)
            self.map_frame = frame
        return store.identity(name, created, self.map_frame, self.robot_frame)

    def _on_state(self, msg):
        self._localized = bool(msg.data)

    def _on_cloud(self, msg):
        self._cloud = msg
        self._cloud_at = rospy.Time.now().to_sec()

    # ------------------------------------------------------------------ the pose
    def current_pose(self):
        """(x, y, yaw, stamp) in the map frame, or raise RuntimeError with the reason."""
        if self.require_localized and self._localized is not True:
            raise RuntimeError(
                "NOT localized: /slam/manager/state is %s. Without it the relocalizer "
                "publishes no %s -> camera_init, so there are no map coordinates to "
                "record. Odometry coordinates are NOT a substitute."
                % ("absent" if self._localized is None else "False", self.map_frame))
        try:
            tfm = self._tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0),
                rospy.Duration(self.tf_timeout))
        except Exception as exc:                            # noqa: BLE001
            raise RuntimeError("no %s -> %s transform (%s: %s)"
                              % (self.map_frame, self.robot_frame,
                                 type(exc).__name__, exc))

        # The frames must be the ones we think they are, so a pose expressed in
        # an odometry frame can never be written into a map-frame file.
        got_parent = tfm.header.frame_id.lstrip("/")
        got_child = tfm.child_frame_id.lstrip("/")
        if got_parent != self.map_frame or got_child != self.robot_frame:
            raise RuntimeError("transform is %s -> %s, expected %s -> %s; refusing to "
                               "record a pose from the wrong frames"
                               % (got_parent, got_child, self.map_frame, self.robot_frame))

        stamp = tfm.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0.0:
            raise RuntimeError("transform carries no usable stamp")
        if age > self.max_tf_age:
            raise RuntimeError("transform is stale: %.3fs old (limit %.3fs). Stand still "
                               "until localisation catches up." % (age, self.max_tf_age))

        t, r = tfm.transform.translation, tfm.transform.rotation
        return t.x, t.y, _yaw_from_quaternion(r.x, r.y, r.z, r.w), stamp

    # ------------------------------------------------------------------ snapshot
    def write_snapshot(self, name, x, y):
        """Crop the latest cloud to a radius about (x, y) and write a .pcd.

        Environmental reference only. It is NOT used to localise anything, and
        no new localisation algorithm is implemented here -- the relocalizer
        already owns that.
        """
        if not self.snapshot:
            return None
        if self._cloud is None:
            rospy.logwarn("[record_waypoints] no cloud on %s yet; saving the pose "
                          "without a snapshot", self.snapshot_topic)
            return None
        age = rospy.Time.now().to_sec() - (self._cloud_at or 0.0)
        if age > max(1.0, self.max_tf_age * 4):
            rospy.logwarn("[record_waypoints] cloud is %.2fs old; saving the pose "
                          "without a snapshot", age)
            return None
        try:
            points = cu.pc2_to_xyz(self._cloud)
        except Exception as exc:                            # noqa: BLE001
            rospy.logwarn("[record_waypoints] cannot read the cloud (%s); pose saved "
                          "without a snapshot", exc)
            return None
        near = [p for p in points
                if (p[0] - x) ** 2 + (p[1] - y) ** 2 <= self.snapshot_radius ** 2]
        if not near:
            rospy.logwarn("[record_waypoints] no points within %.2f m; pose saved "
                          "without a snapshot", self.snapshot_radius)
            return None
        fname = "%s_%s.pcd" % (name, time.strftime("%Y%m%d_%H%M%S"))
        path = os.path.join(os.path.dirname(os.path.abspath(self.waypoints_file)), fname)
        cu.write_pcd(path, near)
        rospy.loginfo("[record_waypoints] snapshot: %s (%d points within %.1f m)",
                      fname, len(near), self.snapshot_radius)
        return fname

    # ------------------------------------------------------------------ actions
    def save(self):
        try:
            x, y, yaw, stamp = self.current_pose()
        except RuntimeError as exc:
            print("  REFUSED: %s" % exc)
            return
        # Name first, so the snapshot can carry it.
        try:
            _ident, existing = (store.load(self.waypoints_file)
                                if os.path.exists(self.waypoints_file) else (None, {}))
        except store.StoreError as exc:
            print("  REFUSED: %s" % exc)
            return
        name = store.next_name(existing, self.prefix)
        snapshot = self.write_snapshot(name, x, y)
        try:
            key, record = store.add(
                self.waypoints_file, self.identity, x, y, yaw,
                recorded_at=datetime.datetime.now().isoformat(timespec="seconds"),
                stamp=stamp, name=name, cloud_snapshot=snapshot)
        except store.MapMismatch as exc:
            print("  REFUSED (map mismatch): %s" % exc)
            return
        except store.StoreError as exc:
            print("  REFUSED: %s" % exc)
            return
        import math
        print("  saved %-16s x=%+.3f y=%+.3f yaw=%+.3f rad (%+.1f deg)%s"
              % (key, record["x"], record["y"], record["yaw"],
                 math.degrees(record["yaw"]),
                 "  + snapshot" if snapshot else ""))

    def listing(self):
        if not os.path.exists(self.waypoints_file):
            print("  (nothing recorded yet: %s)" % self.waypoints_file)
            return
        try:
            ident, waypoints = store.load(self.waypoints_file)
        except store.StoreError as exc:
            print("  cannot read %s: %s" % (self.waypoints_file, exc))
            return
        print("  map %r (created %s), %d record(s):"
              % ((ident or {}).get("name"), (ident or {}).get("created_at"),
                 len(waypoints)))
        import math
        for key in sorted(waypoints):
            w = waypoints[key]
            print("    %-16s x=%+.3f y=%+.3f yaw=%+.3f (%+.1f deg)  %s"
                  % (key, w["x"], w["y"], w["yaw"], math.degrees(w["yaw"]),
                     w.get("recorded_at", "")))

    # ------------------------------------------------------------------ main loop
    def spin(self):
        if not sys.stdin.isatty():
            rospy.logfatal("[record_waypoints] stdin is not a terminal, so no key press "
                           "can be read. Run this from an interactive shell, not from a "
                           "launch file or a pipe.")
            return 1
        print("\nRecording standing poses into:\n  %s" % self.waypoints_file)
        print("map: %s (created %s)" % (self.identity["name"],
                                        self.identity["created_at"]))
        print(HELP)
        print("This never moves the robot.\n")

        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not rospy.is_shutdown():
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                key = ch.lower()
                if key == "a":
                    self.save()
                elif key == "l":
                    self.listing()
                elif key in ("q", "\x03"):          # q or Ctrl-C
                    print("  bye")
                    return 0
                elif key in ("h", "?"):
                    print(HELP)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        return 0


def main():
    try:
        return Recorder().spin()
    except rospy.ROSInitException as exc:
        rospy.logfatal("[record_waypoints] %s", exc)
        return 1
    except rospy.ROSInterruptException:
        return 0


if __name__ == "__main__":
    sys.exit(main())
