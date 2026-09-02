"""Module 1 (Perception) — pixels to base-frame coordinates.

Implements the exact algorithm from the pipeline spec: YOLO bbox -> depth patch
median -> pinhole back-projection -> push along the view ray by the class radius
-> extrinsic transform to the robot base frame. This module has no ROS
dependency; `handle_get_object_position` has the same fields as the spec's
GetObjectPosition.srv and is meant to be called directly from a thin rospy
service wrapper once ROS is available on the robot.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from ultralytics import YOLO

DEPTH_MIN_M = 0.15
DEPTH_MAX_M = 5.0
DEPTH_PATCH_RADIUS_PX = 3  # 7x7 patch

# Radius (m) to push along the view ray from the visible surface to the object's
# geometric center. A class with no surface depth (e.g. a flat button) uses 0.0.
# "bottle" is measured, not a placeholder: the fine-tuned detector (see
# perception/models/bottle_finetuned.pt) was trained on real captured video of
# a standard 330 ml drink can (~0.033 m radius), kept under the label "bottle"
# for consistency with the spec's own running example and this module's
# interface -- it is a can, not a bottle. "cup"/"remote" and the default
# below are still unverified placeholders for the stock pretrained detector.
CLASS_RADIUS_M = {
    "bottle": 0.033,
    "cup": 0.04,  # placeholder, not measured
    "remote": 0.01,  # placeholder, not measured
}
DEFAULT_RADIUS_M = 0.02  # placeholder for any other class


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_K(cls, K: np.ndarray) -> "CameraIntrinsics":
        K = np.asarray(K).reshape(3, 3)
        return cls(fx=float(K[0, 0]), fy=float(K[1, 1]), cx=float(K[0, 2]), cy=float(K[1, 2]))


def _default_R_base_opt() -> np.ndarray:
    # local X/Y axes of the D435i mount, in pelvis-frame directions, from
    # VisualMimic's published MJCF (github.com/visualmimic/VisualMimic,
    # asset/g1/lift_box.xml): the d435_link body under torso_link has
    # xyaxes="-0.008 -1.000 0.000  0.724 -0.006 0.690" (MuJoCo camera-local
    # convention: X-right/Y-up/looks along -Z). Converting to the OpenCV/ROS
    # optical convention (X-right/Y-down/Z-forward) used by backproject_pixel
    # is a 180 deg rotation about local X, i.e. diag(1,-1,-1).
    local_x = np.array([-0.008, -1.000, 0.000])
    local_y = np.array([0.724, -0.006, 0.690])
    local_z = np.cross(local_x, local_y)
    R_world_camlocal = np.stack([local_x, local_y, local_z], axis=1)
    return R_world_camlocal @ np.diag([1.0, -1.0, -1.0])


@dataclass
class Extrinsics:
    """Analytic optical-frame -> base-frame transform (hard-coded, not TF).

    Per spec: R_base_opt = R(camera mounting yaw/pitch/roll) @ R_optical_to_link,
    where R_optical_to_link is the fixed (-90, 0, -90) deg ypr rotation from the
    camera's optical convention (z forward, x right, y down) to a link-style
    convention (x forward, y left, z up).

    Defaults below (~46.4 deg downward pitch, ~(0.054, 0.018, 0.474) m from
    base_link/pelvis at zero waist-joint pose) come from VisualMimic's
    published MJCF for a head-mounted D435i on this exact G1 (see
    _default_R_base_opt's docstring) -- a real, deployed sim-to-real setup,
    not a guess. It independently agrees with two rougher estimates we made
    before finding it (a ~47.6 deg reading off a third-party FOV diagram, and
    a ~(0.055, 0.001, 0.463) m position estimate from the official G1 URDF's
    head mesh), which is reassuring, but it is still someone else's physical
    mounting, not this team's -- confirm against the real robot once
    available, especially if the mount differs from VisualMimic's rig.
    """

    R_base_opt: np.ndarray = field(default_factory=_default_R_base_opt)
    t_base_opt: np.ndarray = field(default_factory=lambda: np.array([0.05366, 0.01753, 0.47387]))


@dataclass
class ObjectPositionResult:
    success: bool
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    frame_id: str = "base_link"
    message: str = ""


def bbox_center(xyxy: np.ndarray) -> tuple[int, int]:
    x1, y1, x2, y2 = xyxy
    return int(round((x1 + x2) / 2)), int(round((y1 + y2) / 2))


def median_nonzero_depth(depth_m: np.ndarray, u: int, v: int, patch: int = DEPTH_PATCH_RADIUS_PX) -> float | None:
    h, w = depth_m.shape[:2]
    u0, u1 = max(0, u - patch), min(w, u + patch + 1)
    v0, v1 = max(0, v - patch), min(h, v + patch + 1)
    window = depth_m[v0:v1, u0:u1]
    valid = window[window > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def backproject_pixel(u: int, v: int, z_c: float, K: CameraIntrinsics) -> np.ndarray:
    x_c = (u - K.cx) * z_c / K.fx
    y_c = (v - K.cy) * z_c / K.fy
    return np.array([x_c, y_c, z_c], dtype=np.float64)


def push_along_ray(p_optical: np.ndarray, radius_m: float) -> np.ndarray:
    norm = np.linalg.norm(p_optical)
    if norm == 0:
        return p_optical
    return p_optical + (p_optical / norm) * radius_m


def optical_to_base(p_optical: np.ndarray, extrinsics: Extrinsics) -> np.ndarray:
    return extrinsics.R_base_opt @ p_optical + extrinsics.t_base_opt


@dataclass
class Detection:
    label: str
    score: float
    p_base: np.ndarray


def run_detection_pipeline(
    rgb: np.ndarray,
    depth_mm: np.ndarray,
    model: YOLO,
    K: CameraIntrinsics,
    extrinsics: Extrinsics,
    conf: float = 0.40,
    iou: float = 0.45,
) -> list[Detection]:
    """Steps 1-5 of the spec's algorithm, run once on one RGB/depth pair."""
    depth_m = depth_mm.astype(np.float64) * 0.001
    results = model.predict(rgb, conf=conf, iou=iou, verbose=False)[0]

    records: list[Detection] = []
    for box in results.boxes:
        label = results.names[int(box.cls.item())]
        score = float(box.conf.item())
        u, v = bbox_center(box.xyxy[0].cpu().numpy())

        z_c = median_nonzero_depth(depth_m, u, v)
        if z_c is None or not (DEPTH_MIN_M <= z_c <= DEPTH_MAX_M):
            continue

        p_optical = backproject_pixel(u, v, z_c, K)
        radius = CLASS_RADIUS_M.get(label, DEFAULT_RADIUS_M)
        p_optical = push_along_ray(p_optical, radius)
        p_base = optical_to_base(p_optical, extrinsics)

        records.append(Detection(label=label, score=score, p_base=p_base))

    return records


def handle_get_object_position(
    rgb: np.ndarray,
    depth_mm: np.ndarray,
    model: YOLO,
    K: CameraIntrinsics,
    extrinsics: Extrinsics,
    label: str,
    index: int = 0,
    frame_id: str = "base_link",
) -> ObjectPositionResult:
    """Mirrors GetObjectPosition.srv: same request/response fields."""
    records = run_detection_pipeline(rgb, depth_mm, model, K, extrinsics)

    matches = [r for r in records if not label or r.label == label]
    matches.sort(key=lambda r: -r.p_base[2])

    if index >= len(matches):
        return ObjectPositionResult(success=False, message=f"no match for label={label!r} at index={index}")

    p = matches[index].p_base
    return ObjectPositionResult(success=True, x=float(p[0]), y=float(p[1]), z=float(p[2]), frame_id=frame_id)
