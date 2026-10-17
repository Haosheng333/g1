"""Module 1 (Perception) — pixels to base-frame coordinates.

Implements the exact algorithm from the pipeline spec: YOLO bbox -> depth patch
median -> pinhole back-projection -> push along the view ray by the class radius
-> extrinsic transform to the robot base frame (``torso_link``). This module has no ROS
dependency; `handle_get_object_position` has the same fields as the spec's
GetObjectPosition.srv and is meant to be called directly from a thin rospy
service wrapper once ROS is available on the robot.
"""

from __future__ import annotations

import math
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


#: Frame every returned point is expressed in. It is the frame the camera is
#: rigidly mounted on, so the transform below holds at every waist angle, and it
#: is the frame the arm planners (arm_grasp, arm_planning) take targets in.
BASE_FRAME = "torso_link"

#: torso_link -> camera_link mount, calibrated on this robot in g1_intellect
#: (g1_camera/config/params.yaml, where the same physical frame is labelled
#: ``base_link``). A constant offset in detected positions is a translation
#: error; a distance-dependent one is a rotation error, so retune the pitch.
CAMERA_TRANSLATION = (0.0576235, 0.02753, 0.44487)
CAMERA_ROTATION_YPR = (0.0, 0.8307767239493009, 0.0)  # yaw, pitch, roll (rad)


def euler_ypr_to_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), right-handed, ROS convention."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    return Rz @ Ry @ Rx


#: camera_color_optical_frame (x right, y down, z forward) -> camera_link
#: (x forward, y left, z up): the fixed (-90, 0, -90) deg ypr rotation.
R_OPTICAL_TO_LINK = euler_ypr_to_matrix(-math.pi / 2, 0.0, -math.pi / 2)


def extrinsics_from_mount(translation, rotation_ypr) -> "Extrinsics":
    """Build the optical -> base transform from a base -> camera_link mount."""
    R_base_cam = euler_ypr_to_matrix(*(float(a) for a in rotation_ypr))
    return Extrinsics(
        R_base_opt=R_base_cam @ R_OPTICAL_TO_LINK,
        t_base_opt=np.asarray(translation, dtype=np.float64).reshape(3),
    )


def _default_R_base_opt() -> np.ndarray:
    return euler_ypr_to_matrix(*CAMERA_ROTATION_YPR) @ R_OPTICAL_TO_LINK


@dataclass
class Extrinsics:
    """Analytic optical-frame -> torso_link transform (hard-coded, not TF).

    R_base_opt = R(camera mounting yaw/pitch/roll) @ R_OPTICAL_TO_LINK, both
    proper rotations. Defaults are the mount calibrated on this robot
    (CAMERA_TRANSLATION / CAMERA_ROTATION_YPR); the node can override them.
    """

    R_base_opt: np.ndarray = field(default_factory=_default_R_base_opt)
    t_base_opt: np.ndarray = field(default_factory=lambda: np.array(CAMERA_TRANSLATION, dtype=np.float64))


@dataclass
class ObjectPositionResult:
    success: bool
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    frame_id: str = BASE_FRAME
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
    frame_id: str = BASE_FRAME,
) -> ObjectPositionResult:
    """Mirrors GetObjectPosition.srv: same request/response fields."""
    records = run_detection_pipeline(rgb, depth_mm, model, K, extrinsics)

    matches = [r for r in records if not label or r.label == label]
    matches.sort(key=lambda r: -r.p_base[2])

    if index >= len(matches):
        return ObjectPositionResult(success=False, message=f"no match for label={label!r} at index={index}")

    p = matches[index].p_base
    return ObjectPositionResult(success=True, x=float(p[0]), y=float(p[1]), z=float(p[2]), frame_id=frame_id)
