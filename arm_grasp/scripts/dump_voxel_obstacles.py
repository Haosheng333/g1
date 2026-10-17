#!/usr/bin/env python3
"""
Snapshot the latched /g1_arm_simple/voxel_obstacles cloud into files.

Usage:
  rosrun arm_grasp dump_voxel_obstacles.py
  # writes /tmp/voxel_obstacles.npy  (N,3 float32, torso_link frame)
  #        /tmp/voxel_obstacles.csv  (x,y,z per line)
  #        /tmp/voxel_indices.npy    (N,3 int64, planner grid indices)
  #        /tmp/voxel_meta.txt       (voxel_size, grid_origin)

Reads voxel_size and grid_origin from voxel_phase2_<arm>.json (currently
voxel_size=0.01). Picks right arm by default; override with --arm left.
"""
import argparse
import json
import os
import sys

import numpy as np
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

_PKG_VOXEL_OUTPUTS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "models", "abstraction_map",
)


def load_grid_meta(arm):
    meta_path = os.path.join(_PKG_VOXEL_OUTPUTS, f"voxel_phase2_{arm}.json")
    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)
    grid_origin = np.array(meta["grid_origin"], dtype=np.float64)
    voxel_size = float(meta["voxel_size"])
    return grid_origin, voxel_size


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="right", choices=["left", "right"])
    ap.add_argument("--topic", default="/g1_arm_simple/voxel_obstacles")
    ap.add_argument("--out-dir", default="/tmp")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    rospy.init_node("dump_voxel_obstacles", anonymous=True)
    try:
        msg = rospy.wait_for_message(args.topic, PointCloud2, timeout=args.timeout)
    except rospy.ROSException:
        print(f"ERROR: no message on {args.topic} within {args.timeout}s", file=sys.stderr)
        sys.exit(1)

    pts = np.asarray(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                     dtype=np.float32)
    print(f"received {pts.shape[0]} points in frame {msg.header.frame_id}")

    grid_origin, voxel_size = load_grid_meta(args.arm)
    idx = np.floor((pts.astype(np.float64) - grid_origin) / voxel_size).astype(np.int64)

    os.makedirs(args.out_dir, exist_ok=True)
    np.save(os.path.join(args.out_dir, "voxel_obstacles.npy"), pts)
    np.savetxt(os.path.join(args.out_dir, "voxel_obstacles.csv"), pts, delimiter=",", header="x,y,z", comments="")
    np.save(os.path.join(args.out_dir, "voxel_indices.npy"), idx)
    with open(os.path.join(args.out_dir, "voxel_meta.txt"), "w") as f:
        f.write(f"arm={args.arm}\n")
        f.write(f"frame={msg.header.frame_id}\n")
        f.write(f"stamp={msg.header.stamp}\n")
        f.write(f"voxel_size={voxel_size}\n")
        f.write(f"grid_origin={grid_origin.tolist()}\n")
        f.write(f"num_points={pts.shape[0]}\n")
    print(f"wrote to {args.out_dir}/voxel_obstacles.{{npy,csv}} and voxel_indices.npy")


if __name__ == "__main__":
    main()
