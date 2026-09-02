#!/usr/bin/env python3
"""ROS1 node for Module 1: exposes /perception/get_object_position.

Untested on this machine (no rospy; ROS1 Noetic does not support macOS,
especially Apple Silicon) -- build and run on the robot / an Ubuntu+ROS
Noetic machine: `catkin build perception && source devel/setup.bash &&
rosrun perception perception_node.py`.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from perception.m1_perception import CameraIntrinsics, Extrinsics, handle_get_object_position
from perception.srv import GetObjectPosition, GetObjectPositionResponse
from ultralytics import YOLO

# TODO: confirm against the real robot (spec pitfall #4 -- hard-code this
# analytically, do not query TF). Extrinsics() defaults to a real, published
# D435i-on-G1 mount (VisualMimic's MJCF) rather than a guess -- see its
# docstring in m1_perception.py -- but it's someone else's physical mount,
# not necessarily this team's; re-measure once the robot is available.
EXTRINSICS = Extrinsics()

# Fine-tuned on real captured video of the actual target object (a drink can,
# labeled "bottle" -- see CLASS_RADIUS_M's comment in m1_perception.py).
# Falls back to the stock pretrained model if the fine-tuned weights are
# missing, e.g. if the target object has since changed.
MODEL_PATH = Path(__file__).resolve().parent.parent / "models" / "bottle_finetuned.pt"

WAIT_TIMEOUT_S = 1.0


class PerceptionNode:
    def __init__(self) -> None:
        model_path = MODEL_PATH if MODEL_PATH.exists() else "yolov8n.pt"
        self.model = YOLO(str(model_path))
        self.bridge = CvBridge()
        self._K: CameraIntrinsics | None = None
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self._on_camera_info)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._K is None:  # cache once at startup, per the spec
            self._K = CameraIntrinsics.from_K(np.array(msg.K))

    def handle_request(self, req) -> GetObjectPositionResponse:
        if self._K is None:
            return GetObjectPositionResponse(success=False, message="camera_info not yet received")

        try:
            rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=WAIT_TIMEOUT_S)
            depth_msg = rospy.wait_for_message(
                "/camera/aligned_depth_to_color/image_raw", Image, timeout=WAIT_TIMEOUT_S
            )
        except rospy.ROSException as exc:
            return GetObjectPositionResponse(success=False, message=f"frame wait timed out: {exc}")

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")

        result = handle_get_object_position(
            rgb, depth_mm, self.model, self._K, EXTRINSICS, label=req.label, index=req.index
        )
        return GetObjectPositionResponse(
            success=result.success,
            x=result.x,
            y=result.y,
            z=result.z,
            frame_id=result.frame_id,
            message=result.message,
        )


def main() -> None:
    rospy.init_node("perception_node")
    node = PerceptionNode()
    rospy.Service("/perception/get_object_position", GetObjectPosition, node.handle_request)
    rospy.spin()


if __name__ == "__main__":
    main()
