#!/usr/bin/env python3
"""Interactive demo for perception.m1_perception -- run this on any photo.

No RealSense needed: pass --depth for a synthetic flat depth plane (meters),
or --depth-image for a real 16-bit depth PNG (millimeters) if you have one.

Usage:
    rosrun perception demo_m1.py --image path/to/photo.jpg --label bottle
    rosrun perception demo_m1.py --image path/to/photo.jpg --label bottle --depth 0.8
    rosrun perception demo_m1.py --image path/to/photo.jpg  # no --label: lists everything YOLO sees

(Or run directly with `python3 demo_m1.py ...` from this scripts/ dir without
ROS, as long as `perception` is on PYTHONPATH -- e.g. PYTHONPATH=../src.)
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from perception.m1_perception import (
    CameraIntrinsics,
    Extrinsics,
    handle_get_object_position,
    run_detection_pipeline,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive Module 1 perception demo")
    parser.add_argument("--image", required=True, help="Path to an RGB photo")
    parser.add_argument("--label", default="", help="COCO class to look for (empty = show all)")
    parser.add_argument("--index", type=int, default=0, help="Which match to return (0-based)")
    parser.add_argument("--depth", type=float, default=1.0, help="Synthetic flat depth plane, meters")
    parser.add_argument("--depth-image", default=None, help="Real 16-bit depth PNG (millimeters) instead")
    parser.add_argument(
        "--g1-extrinsics", action="store_true", help="Use the real G1+D435i mount instead of identity"
    )
    parser.add_argument("--fx", type=float, default=615.0, help="Focal length x (px); 615 ~ a typical D435i at 640x480")
    parser.add_argument("--fy", type=float, default=615.0, help="Focal length y (px)")
    parser.add_argument("--conf", type=float, default=0.40, help="YOLO confidence threshold")
    parser.add_argument("--out", default=None, help="Where to save the annotated preview (default: <image>_preview.jpg)")
    args = parser.parse_args()

    rgb = cv2.imread(args.image)
    if rgb is None:
        raise FileNotFoundError(f"Could not read image: {args.image}")
    h, w = rgb.shape[:2]

    if args.depth_image:
        depth_mm = cv2.imread(args.depth_image, cv2.IMREAD_UNCHANGED)
        if depth_mm is None:
            raise FileNotFoundError(f"Could not read depth image: {args.depth_image}")
    else:
        depth_mm = np.full((h, w), int(args.depth * 1000), dtype=np.uint16)
        print(f"[info] no --depth-image given: using a synthetic flat plane at {args.depth:.2f} m")

    K = CameraIntrinsics(fx=args.fx, fy=args.fy, cx=w / 2, cy=h / 2)
    if args.g1_extrinsics:
        extrinsics = Extrinsics()  # the real G1+D435i mount (see its docstring)
    else:
        # explicit identity: report in the camera optical frame, since this
        # demo may be run on any arbitrary photo, not necessarily G1's camera
        extrinsics = Extrinsics(R_base_opt=np.eye(3), t_base_opt=np.zeros(3))

    model = YOLO("yolov8n.pt")

    if not args.label:
        detections = run_detection_pipeline(rgb, depth_mm, model, K, extrinsics, conf=args.conf)
        print(f"\n{len(detections)} object(s) with valid depth found:")
        for d in detections:
            print(f"  {d.label:<12} conf={d.score:.2f}  xyz=({d.p_base[0]:+.3f}, {d.p_base[1]:+.3f}, {d.p_base[2]:.3f}) m")
    else:
        result = handle_get_object_position(rgb, depth_mm, model, K, extrinsics, label=args.label, index=args.index)
        print()
        if result.success:
            print(f"success=True  label={args.label!r}  index={args.index}")
            print(f"xyz = ({result.x:+.3f}, {result.y:+.3f}, {result.z:.3f}) m   frame_id={result.frame_id!r}")
        else:
            print(f"success=False  message={result.message!r}")

    # Re-run once more to get pixel-space boxes for drawing (run_detection_pipeline
    # only returns base-frame xyz, not the original bbox pixels).
    preview = rgb.copy()
    result_boxes = model.predict(rgb, conf=args.conf, verbose=False)[0]
    for box in result_boxes.boxes:
        label = result_boxes.names[int(box.cls.item())]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        color = (0, 255, 0) if label == args.label else (0, 0, 255)
        cv2.rectangle(preview, (x1, y1), (x2, y2), color, 2)
        cv2.putText(preview, f"{label} {box.conf.item():.2f}", (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    out_path = Path(args.out) if args.out else Path(args.image).with_name(Path(args.image).stem + "_preview.jpg")
    cv2.imwrite(str(out_path), preview)
    print(f"\n[saved] annotated preview -> {out_path}")


if __name__ == "__main__":
    main()
