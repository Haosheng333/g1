#!/usr/bin/env python3
"""Bench stand-in for the RealSense: publishes the three topics perception_node
subscribes to, with a synthetic flat depth plane and an optional photo, so the
node and its service can be exercised without the camera.

    rosrun perception fake_camera_publisher.py [--image photo.jpg] [--depth 1.4] [--fps 15]
"""
import argparse

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default="", help="photo to publish (default: flat grey 1280x720)")
    ap.add_argument("--depth", type=float, default=1.4, help="flat depth plane, m")
    ap.add_argument("--fps", type=float, default=15.0)
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("fake_camera")

    if args.image:
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit(f"cannot read {args.image}")
    else:
        bgr = np.full((720, 1280, 3), 128, np.uint8)
    h, w = bgr.shape[:2]
    depth = np.full((h, w), int(args.depth * 1000), np.uint16)

    pub_rgb = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
    pub_depth = rospy.Publisher("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1)
    pub_info = rospy.Publisher("/camera/color/camera_info", CameraInfo, queue_size=1)

    info = CameraInfo(width=w, height=h, distortion_model="plumb_bob")
    fx = 646.0 * w / 1280.0
    info.K = [fx, 0, w / 2, 0, fx, h / 2, 0, 0, 1]
    info.P = [fx, 0, w / 2, 0, 0, fx, h / 2, 0, 0, 0, 1, 0]

    rate = rospy.Rate(args.fps)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        m = Image(height=h, width=w, encoding="rgb8", is_bigendian=0, step=w * 3, data=bgr[:, :, ::-1].tobytes())
        m.header.stamp = stamp; m.header.frame_id = "camera_color_optical_frame"
        d = Image(height=h, width=w, encoding="16UC1", is_bigendian=0, step=w * 2, data=depth.tobytes())
        d.header = m.header
        info.header = m.header
        pub_rgb.publish(m); pub_depth.publish(d); pub_info.publish(info)
        rate.sleep()


if __name__ == "__main__":
    main()
