#!/usr/bin/env python3
"""Live view of what perception sees: the robot's camera image with every
detection drawn on it (box, label, confidence, sampled depth, and the object
centre in torso_link), next to the depth image with the same boxes.

Publishes /perception/annotated_image (bgr8). perception.launch opens it in an
image_view window (viewer_window:=true); it can also be watched with
    rosrun image_view image_view image:=/perception/annotated_image
    rosrun rqt_image_view rqt_image_view /perception/annotated_image
or added to RViz as an Image display.

Runs its own detector pass at ~rate Hz (default 5) and never touches the
service node, so a slow viewer cannot delay /perception/get_object_position.
Same ~model / ~calibration_file / ~camera_* params as perception_node.
"""

from __future__ import annotations

import threading
import time
from collections import deque

import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

from perception.m1_perception import BASE_FRAME, CameraIntrinsics, run_detection_pipeline
from perception.overlay import draw_detections, side_by_side
from perception.ros_image import image_to_array
from perception.ros_setup import load_extrinsics, load_model

SYNC_SLOP_S = 0.10  # see perception_node.py
#: Displayed xyz is the median of the last N frames of the same object (a box
#: of the same label within TRACK_RADIUS_PX of the previous one), like the
#: service's multi-frame median; raw per-frame values flicker by ~1 cm on a
#: shiny can because of depth noise.
SMOOTH_FRAMES = 5
TRACK_RADIUS_PX = 60


class Smoother:
    """Per-object running median of p_base, keyed by label + box position."""

    def __init__(self, n: int = SMOOTH_FRAMES, radius_px: float = TRACK_RADIUS_PX) -> None:
        self.n, self.radius = n, radius_px
        self.tracks: list[dict] = []  # {label, uv, hist: deque}

    def update(self, detections):
        new_tracks = []
        for d in detections:
            if d.p_base is None:
                continue
            best, best_dist = None, self.radius
            for t in self.tracks:
                if t["label"] != d.label or t in new_tracks:
                    continue
                dist = float(np.hypot(t["uv"][0] - d.uv[0], t["uv"][1] - d.uv[1]))
                if dist < best_dist:
                    best, best_dist = t, dist
            if best is None:
                best = {"label": d.label, "hist": deque(maxlen=self.n)}
            best["uv"] = d.uv
            best["hist"].append(np.asarray(d.p_base, dtype=float))
            d.p_base = np.median(np.array(best["hist"]), axis=0)
            new_tracks.append(best)
        self.tracks = new_tracks
        return detections


def bgr_to_imgmsg(bgr: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(bgr).tobytes()
    return msg


class PerceptionViewer:
    def __init__(self) -> None:
        self.model = load_model("viewer")
        self.extrinsics = load_extrinsics("viewer")
        self.rate_hz = float(rospy.get_param("~rate", 5.0))
        self.target_label = str(rospy.get_param("~target_label", "bottle"))
        self.show_depth = bool(rospy.get_param("~show_depth", True))
        self.depth_max_m = float(rospy.get_param("~depth_max_m", 3.0))
        self.conf = float(rospy.get_param("~conf", 0.40))
        self.smoother = Smoother(max(1, int(rospy.get_param("~smooth_frames", SMOOTH_FRAMES))))
        self.offset = np.asarray(rospy.get_param("~offset_xyz", [0.0, 0.0, 0.0]), dtype=float).reshape(3)

        self._K: CameraIntrinsics | None = None
        self._lock = threading.Lock()
        self._pair: tuple[Image, Image] | None = None
        self._last_stamp = None

        self.pub = rospy.Publisher("/perception/annotated_image", Image, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self._on_camera_info)
        rgb_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self._sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=SYNC_SLOP_S)
        self._sync.registerCallback(self._on_pair)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._K = CameraIntrinsics.from_K(np.array(msg.K))

    def _on_pair(self, rgb_msg: Image, depth_msg: Image) -> None:
        with self._lock:
            self._pair = (rgb_msg, depth_msg)

    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        warned = 0.0
        while not rospy.is_shutdown():
            with self._lock:
                pair = self._pair
            if pair is None or self._K is None:
                if time.time() - warned > 5.0:
                    rospy.logwarn("[viewer] waiting for /camera/color/image_raw + aligned depth + camera_info")
                    warned = time.time()
                rate.sleep()
                continue
            rgb_msg, depth_msg = pair
            if rgb_msg.header.stamp == self._last_stamp:
                rate.sleep()
                continue
            self._last_stamp = rgb_msg.header.stamp
            try:
                self._render(rgb_msg, depth_msg)
            except Exception as exc:  # noqa: BLE001
                rospy.logerr_throttle(5.0, "[viewer] %s: %s", type(exc).__name__, exc)
            rate.sleep()

    def _render(self, rgb_msg: Image, depth_msg: Image) -> None:
        t0 = time.time()
        bgr = image_to_array(rgb_msg, 3)
        depth_mm = image_to_array(depth_msg, 1)
        dets = run_detection_pipeline(bgr, depth_mm, self.model, self._K, self.extrinsics,
                                      conf=self.conf, include_invalid_depth=True)
        for d in dets:
            if d.p_base is not None:
                d.p_base = d.p_base - self.offset  # offset_xyz = reported - real
        dets = self.smoother.update(dets)
        located = [d for d in dets if d.p_base is not None]
        hits = [d for d in located if d.label == self.target_label]
        age = (rospy.Time.now() - rgb_msg.header.stamp).to_sec()
        header = (f"looking for '{self.target_label}': {len(hits)} found | {len(located)}/{len(dets)} boxes with depth"
                  f" | frame age {age:.2f}s | {BASE_FRAME}, {self.smoother.n}-frame median")
        out = draw_detections(bgr, dets, target_label=self.target_label, header=header)
        if self.show_depth:
            out = side_by_side(out, depth_mm, dets, self.depth_max_m)
        self.pub.publish(bgr_to_imgmsg(out, rgb_msg.header))
        rospy.logdebug("[viewer] %d detections in %.0f ms", len(dets), (time.time() - t0) * 1000)


def main() -> None:
    rospy.init_node("perception_viewer")
    PerceptionViewer().spin()


if __name__ == "__main__":
    main()
