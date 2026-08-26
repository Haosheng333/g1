"""Batch accuracy evaluation: run the pipeline against MuJoCo-rendered RGB-D
across a grid of object positions (known ground truth), report mean/std/max
error. Same method as mujoco_validation.py (one worked example with detailed
diagnostics); this runs many trials for a summary statistic instead.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from perception.m1_perception import (
    CameraIntrinsics,
    Extrinsics,
    backproject_pixel,
    median_nonzero_depth,
    optical_to_base,
    push_along_ray,
)

WIDTH, HEIGHT = 640, 480
FOVY_DEG = 58.0
CAM_POS = (0.05366, 0.01753, 0.47387)
CAM_XYAXES = "-0.008 -1.000 0.000  0.724 -0.006 0.690"
PELVIS_HEIGHT = 0.793
TABLE_TOP_Z = 0.75 - PELVIS_HEIGHT
OBJECT_RADIUS_M = 0.05
OBJECT_HALF_HEIGHT_M = 0.10

FORWARD_DISTANCES = [0.7, 0.85, 1.0, 1.15, 1.3]
LATERAL_OFFSETS = [-0.15, -0.05, 0.05, 0.15]


def camera_intrinsics_from_fovy(width: int, height: int, fovy_deg: float) -> CameraIntrinsics:
    fy = height / (2 * math.tan(math.radians(fovy_deg) / 2))
    return CameraIntrinsics(fx=fy, fy=fy, cx=width / 2, cy=height / 2)


def build_mjcf(object_pos: tuple[float, float, float]) -> str:
    return f"""
<mujoco>
  <visual><global offwidth="{WIDTH}" offheight="{HEIGHT}"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".3 .4 .5" width="300" height="300"/>
    <material name="grid_mat" texture="grid" texrepeat="4 4" reflectance="0"/>
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" diffuse="1 1 1"/>
    <geom name="floor" type="plane" pos="0 0 -{PELVIS_HEIGHT}" size="3 3 0.1" material="grid_mat"/>
    <geom name="table" type="box" pos="{object_pos[0]} 0 {TABLE_TOP_Z - 0.05}" size="0.6 0.35 0.05" rgba="0.55 0.4 0.25 1"/>
    <camera name="head_cam" pos="{CAM_POS[0]} {CAM_POS[1]} {CAM_POS[2]}" xyaxes="{CAM_XYAXES}" fovy="{FOVY_DEG}"/>
    <body name="bottle" pos="{object_pos[0]} {object_pos[1]} {object_pos[2]}">
      <geom name="bottle_geom" type="cylinder" size="{OBJECT_RADIUS_M} {OBJECT_HALF_HEIGHT_M}" rgba="0.6 0.1 0.1 1"/>
    </body>
  </worldbody>
</mujoco>
"""


def run_trial(object_pos: tuple[float, float, float]) -> tuple[float, bool] | None:
    """Returns (error_m, clipped) where clipped=True if the object's rendered
    silhouette touches the image border (partially out of frame -- biases the
    pixel centroid, since a real detector's bbox would also be cut off there).
    Returns None if the object isn't visible in frame at all.
    """
    model = mujoco.MjModel.from_xml_string(build_mjcf(object_pos))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="head_cam")
    depth_m = renderer.render()
    renderer.disable_depth_rendering()

    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="head_cam")
    seg = renderer.render()
    renderer.disable_segmentation_rendering()

    bottle_id = model.geom("bottle_geom").id
    mask = seg[:, :, 0] == bottle_id
    if not mask.any():
        return None
    vs, us = np.where(mask)
    clipped = bool(vs.min() <= 0 or vs.max() >= HEIGHT - 1 or us.min() <= 0 or us.max() >= WIDTH - 1)
    u, v = int(round(us.mean())), int(round(vs.mean()))

    K = camera_intrinsics_from_fovy(WIDTH, HEIGHT, FOVY_DEG)
    z_c = median_nonzero_depth(depth_m, u, v)
    if z_c is None:
        return None

    p_opt = backproject_pixel(u, v, z_c, K)
    p_opt = push_along_ray(p_opt, OBJECT_RADIUS_M)

    cam_id = model.camera("head_cam").id
    R_wc = data.cam_xmat[cam_id].reshape(3, 3)
    cam_pos_world = data.cam_xpos[cam_id].copy()
    R_wo = R_wc @ np.diag([1.0, -1.0, -1.0])
    ext = Extrinsics(R_base_opt=R_wo, t_base_opt=cam_pos_world)
    p_base = optical_to_base(p_opt, ext)

    error = float(np.linalg.norm(p_base - np.array(object_pos)))
    return error, clipped


def main() -> None:
    in_frame_errors = []
    clipped_errors = []
    not_visible = 0
    print(f"{'distance':>9} {'lateral':>8} {'error_cm':>9}  note")
    for d in FORWARD_DISTANCES:
        for y in LATERAL_OFFSETS:
            object_pos = (d, y, TABLE_TOP_Z + OBJECT_HALF_HEIGHT_M)
            result = run_trial(object_pos)
            if result is None:
                not_visible += 1
                print(f"{d:9.2f} {y:8.2f} {'N/A':>9}  not visible")
                continue
            err, clipped = result
            if clipped:
                clipped_errors.append(err)
                print(f"{d:9.2f} {y:8.2f} {err*100:9.2f}  touches frame edge -- excluded from summary")
            else:
                in_frame_errors.append(err)
                print(f"{d:9.2f} {y:8.2f} {err*100:9.2f}")

    errs = np.array(in_frame_errors)
    print(f"\n{len(errs)} fully-in-frame trials, {len(clipped_errors)} touching the frame edge "
          f"(excluded below), {not_visible} not visible at all")
    print(f"mean error = {errs.mean()*100:.2f} cm")
    print(f"std        = {errs.std()*100:.2f} cm")
    print(f"min/max    = {errs.min()*100:.2f} / {errs.max()*100:.2f} cm")
    print(f"trials < 2cm bar: {(errs < 0.02).sum()}/{len(errs)}")
    if clipped_errors:
        print(f"\nedge-of-frame trials (object silhouette touches the image border, biasing the "
              f"pixel centroid -- a real detector's bbox would be cut off the same way): "
              f"error {min(clipped_errors)*100:.2f}-{max(clipped_errors)*100:.2f} cm. This is a "
              f"framing/FOV limit at this camera's fixed mount, not a math error.")


if __name__ == "__main__":
    main()
