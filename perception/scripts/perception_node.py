#!/usr/bin/env python3
"""ROS1 node for Module 1: exposes /perception/get_object_position.

Run inside the g1_intellect container with the DIP overlay sourced:
    roslaunch perception perception.launch

Image decoding is done with numpy directly from sensor_msgs/Image rather than
cv_bridge: the container's conda env ships numpy 2, and Noetic's cv_bridge is
compiled against numpy 1 (every imgmsg_to_cv2 call raises
"AttributeError: _ARRAY_API not found").
"""

from __future__ import annotations

import threading

import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

from perception.m1_perception import BASE_FRAME, CameraIntrinsics, handle_get_object_position
from perception.ros_image import image_to_array
from perception.ros_setup import load_extrinsics, load_model
from perception.srv import GetObjectPosition, GetObjectPositionResponse

WAIT_TIMEOUT_S = 1.0
#: Consecutive synced frames a call runs the detector on; the per-axis median
#: of the successful ones is returned (at 15 fps, 3 frames ~ 0.2 s).
DEFAULT_FRAMES = 3
#: RGB and aligned depth are paired by ApproximateTimeSynchronizer within this.
#: 0.1 s: over the PC2 link the two streams arrive at different rates
#: (~9 and ~8 Hz) with stamp gaps up to ~70 ms; the scene is static during a
#: call, so a looser pairing costs nothing.
SYNC_SLOP_S = 0.10


class PerceptionNode:
    def __init__(self) -> None:
        self.model = load_model("perception")
        self.extrinsics = load_extrinsics("perception")
        self.frames = max(1, int(rospy.get_param("~frames", DEFAULT_FRAMES)))

        self._K: CameraIntrinsics | None = None
        self._lock = threading.Lock()
        self._pair: tuple[Image, Image] | None = None
        self._pair_event = threading.Event()

        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self._on_camera_info)
        rgb_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self._sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=SYNC_SLOP_S)
        self._sync.registerCallback(self._on_pair)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        K = CameraIntrinsics.from_K(np.array(msg.K))
        if self._K is None or (K.fx, K.fy, K.cx, K.cy) != (self._K.fx, self._K.fy, self._K.cx, self._K.cy):
            self._K = K
            rospy.loginfo("[perception] camera_info: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", K.fx, K.fy, K.cx, K.cy)

    def _on_pair(self, rgb_msg: Image, depth_msg: Image) -> None:
        with self._lock:
            self._pair = (rgb_msg, depth_msg)
        self._pair_event.set()

    def _latest_stamp(self):
        with self._lock:
            return self._pair[0].header.stamp if self._pair is not None else None

    def _fresh_pair(self, after: "rospy.Time | None", timeout: float) -> tuple[Image, Image] | None:
        """Return the first RGB/depth pair whose stamp is newer than `after`.

        `after` is the stamp of the latest pair at request time (None if no
        pair yet), NOT the workstation clock: with the camera on PC2 the frame
        stamps follow PC2's clock, and an offset of even a second would make
        a wall-clock comparison wait forever while the viewer happily shows
        the stream. The timeout is measured on this machine's clock.
        """
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown():
            with self._lock:
                pair = self._pair
            if pair is not None and (after is None or pair[0].header.stamp > after):
                return pair
            remaining = (deadline - rospy.Time.now()).to_sec()
            if remaining <= 0:
                return None
            self._pair_event.clear()
            self._pair_event.wait(min(remaining, 0.1))
        return None

    def handle_request(self, req) -> GetObjectPositionResponse:
        try:
            return self._handle(req)
        except Exception as exc:  # noqa: BLE001 -- report, never drop the service call
            rospy.logerr("[perception] get_object_position failed: %s: %s", type(exc).__name__, exc)
            return GetObjectPositionResponse(success=False, message=f"{type(exc).__name__}: {exc}")

    def _handle(self, req) -> GetObjectPositionResponse:
        if self._K is None:
            return GetObjectPositionResponse(success=False, message="camera_info not yet received")
        if req.index < 0:
            return GetObjectPositionResponse(success=False, message=f"index must be >= 0, got {req.index}")

        after = self._latest_stamp()  # only frames captured after this call started
        results, failures = [], []
        for _ in range(self.frames):
            pair = self._fresh_pair(after, WAIT_TIMEOUT_S)
            if pair is None:
                failures.append(f"no new synchronized RGB/depth pair within {WAIT_TIMEOUT_S:.1f}s "
                                f"(slop {SYNC_SLOP_S:.2f}s); is the RealSense driver running with align_depth?")
                break
            rgb_msg, depth_msg = pair
            after = rgb_msg.header.stamp  # next iteration: a newer frame
            skew = (rospy.Time.now() - rgb_msg.header.stamp).to_sec()
            if abs(skew) > 0.5:
                rospy.logwarn_throttle(10.0, "[perception] frame stamp is %.2f s from this machine's clock: "
                                       "PC2 clock not synced? (frames are still used)", skew)
            rgb = image_to_array(rgb_msg, 3)
            depth_mm = image_to_array(depth_msg, 1)
            if rgb.shape[:2] != depth_mm.shape[:2]:
                return GetObjectPositionResponse(
                    success=False,
                    message=f"rgb {rgb.shape[:2]} and depth {depth_mm.shape[:2]} differ in size; "
                            "the depth topic must be the aligned one")
            r = handle_get_object_position(rgb, depth_mm, self.model, self._K, self.extrinsics,
                                           label=req.label, index=req.index, frame_id=BASE_FRAME)
            (results if r.success else failures).append(r if r.success else r.message)

        if len(results) < (self.frames + 1) // 2:  # need a majority of frames
            return GetObjectPositionResponse(
                success=False,
                message=f"{len(results)}/{self.frames} frames had a match: " + (failures[-1] if failures else ""))
        xyz = np.median(np.array([[r.x, r.y, r.z] for r in results]), axis=0)
        return GetObjectPositionResponse(
            success=True, x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]),
            frame_id=BASE_FRAME, message=f"median of {len(results)}/{self.frames} frames",
        )


def main() -> None:
    rospy.init_node("perception_node")
    node = PerceptionNode()
    rospy.Service("/perception/get_object_position", GetObjectPosition, node.handle_request)
    rospy.loginfo("[perception] /perception/get_object_position ready (frame %s)", BASE_FRAME)
    rospy.spin()


if __name__ == "__main__":
    main()
