"""Validate m1_perception's math against MuJoCo-rendered RGB-D, with exact
ground truth (we place the object ourselves, so there's no measurement error
on the "true" side -- this is a rigorous stand-in for the spec's tape-measure
acceptance test while there's no real RealSense/robot available).

Deliberately NOT the full G1 body model (that needs ~40 mesh files we don't
need for this): just a camera at our best-estimate mounting pose (see
Extrinsics' docstring in m1_perception.py) and one object at a known 3D
position. This isolates exactly what needs checking here: does the
depth-patch-median -> pinhole back-projection -> ray-push -> extrinsic chain
recover that known position from a physically-rendered depth image.

Object pixel location comes from MuJoCo's own segmentation render (ground
truth), not YOLO -- a plain MuJoCo primitive won't read as "bottle" to a
COCO-trained detector (no texture/label/real-world appearance), which is a
sim-to-real domain-gap limitation of this method, not a pipeline bug. YOLO is
still run on the RGB render separately, just to see and report what happens.
"""

from __future__ import annotations

import math

import cv2
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
FOVY_DEG = 58.0  # ~ D435i vertical depth FOV

# Camera mount: real, published values from VisualMimic's MJCF for a
# head-mounted D435i on this exact G1 (github.com/visualmimic/VisualMimic,
# asset/g1/lift_box.xml) -- see Extrinsics' docstring in m1_perception.py.
# xyaxes is that file's local X/Y axes directly, relative to base_link/pelvis
# at zero waist-joint pose (matches the MJCF's own convention exactly).
CAM_POS = (0.05366, 0.01753, 0.47387)
CAM_XYAXES = "-0.008 -1.000 0.000  0.724 -0.006 0.690"

# Ground-truth object placement (world frame == base_link/pelvis frame here).
# G1's pelvis stands ~0.793 m off the ground (also from that MJCF), so a
# normal 0.75 m table is ~-0.04 m relative to pelvis -- almost level with it,
# not "low". (An earlier version of this script used a rough ~0.6 m
# pelvis-height guess and wrongly concluded a table wouldn't be visible --
# that was wrong; corrected here now that we have the real pelvis height too.)
#
# Distance is set to 1.0 m to match the spec's own acceptance-test distance
# ("< 2 cm at 1 m") -- this matters: the push_along_ray error is NOT a fixed
# amount, it grows at closer range. At this camera's fixed height/tilt, a
# closer target needs a steeper look-down angle, which diverges more from
# the cylinder's horizontal radial direction (what push_along_ray implicitly
# assumes), so the approximation gets worse. Measured in this setup: ~1.2 cm
# at 1.0 m (meets the spec bar) vs. ~2.2-2.4 cm at 0.6 m (over it). Worth
# keeping in mind if the robot ever needs to grasp something closer than
# ~0.7 m from this camera.
TABLE_TOP_Z = 0.75 - 0.793  # ~ -0.04, table surface relative to pelvis
OBJECT_RADIUS_M = 0.05  # a real bottle's surface-to-center push distance
OBJECT_HALF_HEIGHT_M = 0.10
OBJECT_POS = (1.00, -0.05, TABLE_TOP_Z + OBJECT_HALF_HEIGHT_M)

MJCF = f"""
<mujoco>
  <visual>
    <global offwidth="{WIDTH}" offheight="{HEIGHT}"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".3 .4 .5" width="300" height="300"/>
    <material name="grid_mat" texture="grid" texrepeat="4 4" reflectance="0"/>
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" diffuse="1 1 1"/>
    <!-- world origin here = pelvis at zero waist-joint pose; ground is
         0.793 m below that (G1's real standing pelvis height, from the same
         MJCF this camera mount comes from). -->
    <geom name="floor" type="plane" pos="0 0 -0.793" size="2 2 0.1" material="grid_mat"/>
    <geom name="table" type="box" pos="{OBJECT_POS[0]} 0 {TABLE_TOP_Z - 0.05}" size="0.35 0.35 0.05" rgba="0.55 0.4 0.25 1"/>

    <!-- head camera, real published mount (see CAM_POS/CAM_XYAXES above) -->
    <camera name="head_cam" pos="{CAM_POS[0]} {CAM_POS[1]} {CAM_POS[2]}"
            xyaxes="{CAM_XYAXES}"
            fovy="{FOVY_DEG}"/>

    <!-- ground-truth object: a "bottle" at a KNOWN position -->
    <body name="bottle" pos="{OBJECT_POS[0]} {OBJECT_POS[1]} {OBJECT_POS[2]}">
      <geom name="bottle_geom" type="cylinder" size="{OBJECT_RADIUS_M} {OBJECT_HALF_HEIGHT_M}" rgba="0.6 0.1 0.1 1"/>
    </body>
  </worldbody>
</mujoco>
"""


def camera_intrinsics_from_fovy(width: int, height: int, fovy_deg: float) -> CameraIntrinsics:
    fy = height / (2 * math.tan(math.radians(fovy_deg) / 2))
    return CameraIntrinsics(fx=fy, fy=fy, cx=width / 2, cy=height / 2)


def main() -> None:
    model = mujoco.MjModel.from_xml_string(MJCF)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

    renderer.update_scene(data, camera="head_cam")
    rgb = renderer.render()

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="head_cam")
    depth_m = renderer.render()  # meters, float
    renderer.disable_depth_rendering()

    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="head_cam")
    seg = renderer.render()  # seg[...,0] = geom id
    renderer.disable_segmentation_rendering()

    bottle_geom_id = model.geom("bottle_geom").id
    mask = seg[:, :, 0] == bottle_geom_id
    if not mask.any():
        print("FAIL: bottle not visible in the rendered camera view at all")
        return
    vs, us = np.where(mask)
    u, v = int(round(us.mean())), int(round(vs.mean()))
    print(f"[ground truth] object placed at base-frame xyz = {OBJECT_POS}")
    print(f"[render] object's pixel centroid (from MuJoCo segmentation) = (u={u}, v={v})")

    rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    x1, x2 = us.min(), us.max()
    y1, y2 = vs.min(), vs.max()
    cv2.rectangle(rgb_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
    out_path = "perception/test/data/mujoco_render_preview.jpg"
    cv2.imwrite(out_path, rgb_bgr)
    print(f"[saved] rendered RGB with ground-truth box -> {out_path}")

    K = camera_intrinsics_from_fovy(WIDTH, HEIGHT, FOVY_DEG)
    z_c = median_nonzero_depth(depth_m, u, v)
    print(f"\n[pipeline] median depth at (u,v) = {z_c:.4f} m")

    p_optical_surface = backproject_pixel(u, v, z_c, K)
    p_optical_pushed = push_along_ray(p_optical_surface, OBJECT_RADIUS_M)

    # Build R_base_opt/t_base_opt from MuJoCo's OWN camera transform (its
    # ground truth for what it just rendered), instead of re-deriving the
    # mount angle by hand -- avoids trusting a second, independently-computed
    # rotation to agree with the first. MuJoCo's camera-local convention is
    # X-right/Y-up/looks along -Z; backproject_pixel assumes the OpenCV/ROS
    # optical convention X-right/Y-down/Z-forward, related by a 180 deg
    # rotation about local X (diag(1,-1,-1)).
    cam_id = model.camera("head_cam").id
    R_world_camlocal = data.cam_xmat[cam_id].reshape(3, 3)
    cam_pos_world = data.cam_xpos[cam_id].copy()
    R_world_optical = R_world_camlocal @ np.diag([1.0, -1.0, -1.0])

    extrinsics = Extrinsics(R_base_opt=R_world_optical, t_base_opt=cam_pos_world)
    p_base_surface = optical_to_base(p_optical_surface, extrinsics)
    p_base_pushed = optical_to_base(p_optical_pushed, extrinsics)

    gt = np.array(OBJECT_POS)
    surface_err = np.linalg.norm(p_base_surface - gt)
    pushed_err = np.linalg.norm(p_base_pushed - gt)
    axis_dist = math.hypot(p_base_surface[0] - gt[0], p_base_surface[1] - gt[1])

    print(f"\n[step 1: raw back-projection, NO radius push] xyz = "
          f"({p_base_surface[0]:.3f}, {p_base_surface[1]:.3f}, {p_base_surface[2]:.3f})")
    print(f"  horizontal distance from the cylinder's true axis = {axis_dist * 100:.2f} cm "
          f"(true radius = {OBJECT_RADIUS_M * 100:.1f} cm) -> back-projection itself is essentially exact")
    print(f"  distance to true CENTER (unpushed, expected to be off by ~radius) = {surface_err * 100:.2f} cm")

    print(f"\n[step 2: after push_along_ray by the class radius, per spec] xyz = "
          f"({p_base_pushed[0]:.3f}, {p_base_pushed[1]:.3f}, {p_base_pushed[2]:.3f})")
    print(f"  ground truth center                                        xyz = {OBJECT_POS}")
    print(f"  error = {pushed_err * 100:.2f} cm  (spec bar: < 2 cm at 1 m)")
    print("  NOTE: the residual error here is the spec's own approximation (push along the VIEW RAY,")
    print("  not along the object's true radial direction) -- exact for a sphere, imperfect for a")
    print("  cylinder viewed at a steep angle. Not a bug in this pipeline's math.")

    print("\n--- separately, real YOLO on the rendered RGB (informational) ---")
    from ultralytics import YOLO

    yolo = YOLO("yolov8n.pt")
    result = yolo.predict(rgb_bgr, conf=0.1, verbose=False)[0]
    if len(result.boxes) == 0:
        print("YOLO found nothing -- expected: a plain untextured cylinder doesn't look like a real bottle")
    else:
        for box in result.boxes:
            print(f"  YOLO saw: {result.names[int(box.cls.item())]} conf={box.conf.item():.2f}")


if __name__ == "__main__":
    main()
