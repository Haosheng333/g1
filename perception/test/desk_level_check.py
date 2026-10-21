#!/usr/bin/env python3
"""Levelling check for the camera extrinsics, against a flat table in view.

Back-projects one aligned depth frame into torso_link with the given
calibration and fits a plane to the table region. With correct extrinsics
(robot standing level, waist at zero) the plane is level: a tilt about y is a
pitch error, about x a roll error, and the table height should match a tape
measure. Also prints the height just below the detected can, if any.

    rosrun perception desk_level_check.py [--calibration-file F] [--zmin -0.12 --zmax 0.03]
"""
import argparse
import math

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

from perception.m1_perception import (
    CameraIntrinsics, extrinsics_from_calibration_yaml, run_detection_pipeline, table_height_below_box,
)
from perception.ros_image import image_to_array


def fit_plane(pts):
    for it in range(3):
        c = pts.mean(0)
        n = np.linalg.svd(pts - c, full_matrices=False)[2][2]
        n = n if n[2] > 0 else -n
        res = (pts - c) @ n
        if it < 2:
            pts = pts[np.abs(res) < 3 * res.std() + 1e-4]
    return c, n, res.std(), pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration-file", default="", help="default: the ~calibration_file param of /perception_node")
    ap.add_argument("--zmin", type=float, default=-0.12, help="table z band in torso_link (rough)")
    ap.add_argument("--zmax", type=float, default=0.03)
    ap.add_argument("--xmin", type=float, default=0.25)
    ap.add_argument("--xmax", type=float, default=0.75)
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("desk_level_check", anonymous=True)
    calib = args.calibration_file or rospy.get_param("/perception_node/calibration_file", "")
    if not calib:
        raise SystemExit("no calibration file given and /perception_node/calibration_file is unset")
    ext = extrinsics_from_calibration_yaml(calib)
    K = CameraIntrinsics.from_K(np.array(rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=5).K))
    rgb = image_to_array(rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5), 3)
    dep_mm = image_to_array(rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=5), 1)
    dep = dep_mm.astype(float) * 0.001
    h, w = dep.shape
    vv, uu = np.mgrid[0:h, 0:w]
    P = np.stack([(uu - K.cx) * dep / K.fx, (vv - K.cy) * dep / K.fy, dep], axis=-1)
    Pb = P @ ext.R_base_opt.T + ext.t_base_opt
    m = (dep > 0) & (Pb[..., 2] > args.zmin) & (Pb[..., 2] < args.zmax) & (Pb[..., 0] > args.xmin) & (Pb[..., 0] < args.xmax) & (np.abs(Pb[..., 1]) < 0.3)

    print("calibration: %s" % calib)
    try:
        from ultralytics import YOLO
        from perception.ros_setup import MODEL_PATH
        dets = [d for d in run_detection_pipeline(rgb, dep_mm, YOLO(str(MODEL_PATH)), K, ext) if d.label == "bottle"]
    except Exception as exc:  # noqa: BLE001
        dets = []
        print("(detector not run: %s)" % exc)
    for d in dets:
        x1, y1, x2, y2 = d.bbox
        m[y1 - 5:y2 + 5, x1 - 5:x2 + 5] = False
        zt = table_height_below_box(dep, d.bbox, K, ext)
        print("can: centre %s  table below it %s" % (np.round(d.p_base, 3), "n/a" if zt is None else "%+.4f" % zt))
    pts = Pb[m]
    if len(pts) < 1000:
        raise SystemExit("too few table points (%d) in the z band %.2f..%.2f; adjust --zmin/--zmax" % (len(pts), args.zmin, args.zmax))
    c, n, sd, pts = fit_plane(pts)
    print("table plane: %d pts, mean z %+.4f m, residual %.1f mm" % (len(pts), c[2], sd * 1000))
    print("tilt about y (pitch error) %+.2f deg, about x (roll error) %+.2f deg   (0 = level)"
          % (math.degrees(math.atan2(n[0], n[2])), math.degrees(math.atan2(n[1], n[2]))))
    for lo in np.arange(args.xmin, args.xmax - 1e-9, 0.1):
        sel = pts[(pts[:, 0] >= lo) & (pts[:, 0] < lo + 0.1)]
        if len(sel) > 100:
            print("  table z at x=%.2f-%.2f m: %+.4f (n=%d)" % (lo, lo + 0.1, np.median(sel[:, 2]), len(sel)))
    print("A residual tilt of ~1 deg is within what the robot's own stance and desk flatness give; "
          "judge by the table height at the object's distance against a tape measure.")


if __name__ == "__main__":
    main()
