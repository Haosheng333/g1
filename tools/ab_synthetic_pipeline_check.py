#!/usr/bin/env python3
"""Robot-free check of the REAL pcd_to_map binary with the exact arguments slam_manager passes.

Builds a synthetic z-DOWN room (floor at z=+1.0 like a real camera_init cloud), runs the same
steps as slam_manager._save (floor estimate -> z-up copy -> rosrun g1_slam pcd_to_map ->
tier grid) and asserts walls are occupied, the interior is free, and tiers appear near walls.
Also re-localizes a partial scan against the saved PCD with the real relocalization code.

Run inside the container (needs `source g1_ws/devel/setup.bash --extend`):
    python3 ~/Workspace/DIP/tools/ab_synthetic_pipeline_check.py
"""
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "slam", "test"))
sys.path.insert(0, os.path.join(REPO, "slam", "src"))

from scipy.spatial import cKDTree  # noqa: E402
from g1_slam import cloud_utils as cu  # noqa: E402
from g1_slam import grid_utils as gu  # noqa: E402
from test_cloud_utils import make_room  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        FAILS.append(name)


def cell(grid, res, ox, oy, x, y, r=1):
    i, j = int((x - ox) / res), int((y - oy) / res)
    return grid[max(j - r, 0):j + r + 1, max(i - r, 0):i + r + 1]


def main():
    room = make_room()                                   # camera_init, z-down
    floor_down = cu.estimate_floor_z_down(room)
    check("floor estimate (z-down floor = +1.0)", floor_down is not None and abs(floor_down - 1.0) < 0.06,
          "got %s" % floor_down)
    out = tempfile.mkdtemp(prefix="ab_check_")
    pcd, pcd_up = os.path.join(out, "map.pcd"), os.path.join(out, "map_zup.pcd")
    room_saved = cu.voxel_downsample(room, 0.05)         # slam_manager's default voxel
    cu.write_pcd(pcd, room_saved)
    R = cu.rpy_to_R(math.pi, 0.0, 0.0)
    cu.write_pcd(pcd_up, cu.apply_T(cu.make_T(R, [0, 0, 0]), room_saved))
    fu = -floor_down
    cmd = ["rosrun", "g1_slam", "pcd_to_map", "--input", pcd_up, "--name", "map", "--output-dir", out,
           "--resolution", "0.05", "--z-min", "%.3f" % (fu + 0.15), "--z-max", "%.3f" % (fu + 1.80),
           "--ground-z-min", "%.3f" % (fu - 0.30)]
    print("running:", " ".join(cmd))
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    check("pcd_to_map exits 0", proc.returncode == 0, proc.stderr.strip()[-300:])
    if proc.returncode != 0:
        return finish()
    grid, res, ox, oy = gu.load_map_yaml(os.path.join(out, "map.yaml"))
    print("grid %dx%d @ %.2f origin (%.2f, %.2f)" % (grid.shape[1], grid.shape[0], res, ox, oy))
    # map frame = (x, -y): room spans x 0..8, y' -6..0
    raw_unknown = (grid[int((-6 - oy) / res):int((0 - oy) / res), int((0 - ox) / res):int((8 - ox) / res)]
                   == gu.UNKNOWN).mean()
    print("raw (before hole filling) unknown fraction inside the room: %.0f%%" % (100 * raw_unknown))
    check("wall x=0 at (0,-3) occupied", (cell(grid, res, ox, oy, 0.0, -3.0, 2) == gu.OBSTACLE).any())
    check("partition x=3 at (3,-4.5) occupied", (cell(grid, res, ox, oy, 3.0, -4.5, 2) == gu.OBSTACLE).any())
    check("NOT mirrored: nothing occupied at (3,+4.5)", not (cell(grid, res, ox, oy, 3.0, 4.5, 2) == gu.OBSTACLE).any())
    tiers = gu.compose_tiers(grid, res)
    room_t = tiers[int((-5.5 - oy) / res):int((-0.5 - oy) / res), int((0.5 - ox) / res):int((7.5 - ox) / res)]
    unk = (room_t == gu.UNKNOWN).mean()
    check("interior (1.5,-1.5) walkable after composing", tiers[int((-1.5 - oy) / res), int((1.5 - ox) / res)] == gu.WALKABLE)
    check("room interior has <5% unknown after hole filling", unk < 0.05, "%.1f%%" % (100 * unk))
    check("tier values present", {gu.WALKABLE, gu.RISK, gu.DANGER, gu.OBSTACLE} <= set(np.unique(tiers).tolist()),
          str(np.unique(tiers)))

    # relocalize a partial scan against the saved PCD with the production code
    dst = cu.voxel_downsample(cu.read_pcd_xyz(pcd), 0.25)
    tree = cKDTree(dst)
    T_true = cu.make_T(cu.rpy_to_R(0.04, 0.0, math.radians(-35)), [2.5, 4.0, 0.0])
    live = cu.apply_T(cu.invert_T(T_true), room)
    c = cu.apply_T(cu.invert_T(T_true), np.array([[2.5, 4.0, 0.0]]))[0]
    src = cu.voxel_downsample(live[np.linalg.norm(live[:, :2] - c[:2], axis=1) < 4.5], 0.25)
    T, fit, rmse, runner = cu.register(src, dst, tree)
    e = cu.invert_T(T_true) @ T
    check("relocalization pose error < 0.25 m / 4 deg",
          np.linalg.norm(e[:3, 3]) < 0.25 and abs(math.degrees(cu.yaw_of(e[:3, :3]))) < 4,
          "fit=%.2f rmse=%.3f runner_up=%.2f" % (fit, rmse, runner))
    return finish()


def finish():
    print("\nRESULT:", "ALL PASS" if not FAILS else "FAILED: " + ", ".join(FAILS))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
