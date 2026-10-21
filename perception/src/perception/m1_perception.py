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
DEPTH_PATCH_RADIUS_PX = 3  # 7x7 patch (median_nonzero_depth, kept for callers)
#: Fraction of the box width / height, centred, that the surface depth is the
#: median over. A single 7x7 patch on a shiny can flickered by >2 cm between
#: frames; the central 30% x 50% of the box is hundreds of pixels on the
#: front face while staying off the curved sides (<1 mm of extra range).
DEPTH_BOX_FRAC_W = 0.30
DEPTH_BOX_FRAC_H = 0.50
#: For upright cylinders the surface sample is taken lower in the box (rows
#: 60%-90%), because from a camera looking down the top face can occupy the
#: upper half of the box and a sample there is nearer than the front face.
CYL_SAMPLE_ROWS = (0.60, 0.90)

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

# Classes that are upright cylinders standing on a surface, with a FALLBACK
# height (m). For these the returned point is the TRUE geometric centre: the
# radius is pushed horizontally (not along the downward view ray), and the
# height is MEASURED per detection -- table height from the depth under the
# box, top of the can from the highest depth points inside the box -- so two
# cans of different size return different z. The fallback height (and
# CLASS_RADIUS_M) is used only when the top or table cannot be read from
# depth. Classes not listed here keep the spec's plain ray push.
CLASS_HEIGHT_M = {
    "bottle": 0.14,  # the 14 cm drink can the detector was fine-tuned on
}
#: Accepted measured cylinder sizes; outside -> fallback constants.
HEIGHT_RANGE_M = (0.04, 0.40)
RADIUS_RANGE_M = (0.015, 0.08)
#: Can points must be this far above the table to count towards its top.
TOP_MIN_ABOVE_TABLE_M = 0.015
#: Percentile of can-point heights taken as the top (robust to a few outliers).
TOP_PERCENTILE = 98
#: The points forming the top must reach within this fraction of the box
#: height from its top row, else the top is not observed (-> fallback height).
TOP_OBSERVED_FRAC = 0.25
#: Below this the view ray is treated as vertical and the plain ray push is used.
MIN_HORIZONTAL_RAY = 0.05
#: Table height under a cylinder is read from the depth in a strip just below
#: its box (rows y2+TABLE_STRIP_GAP .. +TABLE_STRIP_ROWS, +-TABLE_STRIP_HALF_W
#: columns about the box centre). The box-bottom solve is the fallback: the
#: detector's box stops a few pixels above the shadowed contact line, which
#: read 2 cm too high on the real can.
TABLE_STRIP_GAP = 3
TABLE_STRIP_ROWS = 12
TABLE_STRIP_HALF_W = 15
TABLE_STRIP_MIN_PTS = 30
TABLE_STRIP_MAX_SPREAD_M = 0.015


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


def extrinsics_from_calibration_yaml(path) -> "Extrinsics":
    """Load a ``d455_extrinsics_<stamp>.yaml`` written by the ChArUco tool
    (``g1_camera_calibration``): its ``parent_to_color_optical`` block is the
    torso_link -> camera_color_optical_frame transform directly, so no
    R_OPTICAL_TO_LINK composition is needed. The file's ``status`` is
    CANDIDATE until a detected point has been checked against a tape measure.
    """
    import yaml

    with open(path) as f:
        doc = yaml.safe_load(f)
    block = doc["parent_to_color_optical"]
    if block.get("parent_frame") != BASE_FRAME:
        raise ValueError(f"{path}: parent_frame is {block.get('parent_frame')!r}, expected {BASE_FRAME!r}")
    M = np.asarray(block["matrix_4x4"], dtype=np.float64).reshape(4, 4)
    R = M[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or abs(np.linalg.det(R) - 1.0) > 1e-6:
        raise ValueError(f"{path}: rotation block is not a proper rotation")
    return Extrinsics(R_base_opt=R, t_base_opt=M[:3, 3].copy())


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


def median_depth_in_box(depth_m: np.ndarray, bbox: tuple[int, int, int, int],
                        frac_w: float = DEPTH_BOX_FRAC_W, frac_h: float = DEPTH_BOX_FRAC_H,
                        rows: tuple[float, float] | None = None) -> float | None:
    """Median of the valid depths in a region of ``bbox``.

    The region is the central ``frac_w`` of the width and either the central
    ``frac_h`` of the height or, with ``rows=(a, b)``, rows a..b of the box
    height. Falls back to the 7x7 patch at the region centre when it has no
    valid depth at all.
    """
    x1, y1, x2, y2 = bbox
    h, w = depth_m.shape[:2]
    cx = (x1 + x2) / 2.0
    hw = max(1.0, (x2 - x1) * frac_w / 2.0)
    if rows is None:
        cy = (y1 + y2) / 2.0
        hh = max(1.0, (y2 - y1) * frac_h / 2.0)
        v0, v1 = max(0, int(cy - hh)), min(h, int(cy + hh) + 1)
    else:
        v0, v1 = max(0, int(y1 + rows[0] * (y2 - y1))), min(h, int(y1 + rows[1] * (y2 - y1)) + 1)
        cy = (v0 + v1 - 1) / 2.0
    u0, u1 = max(0, int(cx - hw)), min(w, int(cx + hw) + 1)
    window = depth_m[v0:v1, u0:u1]
    valid = window[window > 0]
    if valid.size == 0:
        return median_nonzero_depth(depth_m, int(round(cx)), int(round(cy)))
    return float(np.median(valid))


def cylinder_sample_pixel(bbox: tuple[int, int, int, int]) -> tuple[int, int]:
    """Pixel the cylinder surface sample (CYL_SAMPLE_ROWS band) is centred on."""
    x1, y1, x2, y2 = bbox
    v0, v1 = int(y1 + CYL_SAMPLE_ROWS[0] * (y2 - y1)), int(y1 + CYL_SAMPLE_ROWS[1] * (y2 - y1))
    return int(round((x1 + x2) / 2.0)), int(round((v0 + v1) / 2.0))


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


def project_rows(points_base: np.ndarray, K: CameraIntrinsics, extrinsics: Extrinsics) -> np.ndarray:
    """Image row (v) of each base-frame point; NaN for points behind the camera."""
    p_opt = (points_base - extrinsics.t_base_opt) @ extrinsics.R_base_opt  # == R^T (p - t) per row
    z = p_opt[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        v = K.cy + p_opt[:, 1] * K.fy / z
    v[z <= 1e-6] = np.nan
    return v


def _circle(cx: float, cy: float, z: float, r: float, n: int = 72) -> np.ndarray:
    az = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.stack([cx + r * np.cos(az), cy + r * np.sin(az), np.full(n, z)], axis=1)


def table_height_below_box(depth_m: np.ndarray, bbox: tuple[int, int, int, int], K: CameraIntrinsics,
                           extrinsics: Extrinsics) -> float | None:
    """Base-frame z of the surface just below the box, from depth; None if unusable."""
    x1, y1, x2, y2 = bbox
    h, w = depth_m.shape[:2]
    uc = (x1 + x2) // 2
    v0, v1 = y2 + TABLE_STRIP_GAP, min(h, y2 + TABLE_STRIP_GAP + TABLE_STRIP_ROWS)
    u0, u1 = max(0, uc - TABLE_STRIP_HALF_W), min(w, uc + TABLE_STRIP_HALF_W + 1)
    if v1 - v0 < 3:
        return None
    vv, uu = np.mgrid[v0:v1, u0:u1]
    z = depth_m[v0:v1, u0:u1]
    ok = (z > DEPTH_MIN_M) & (z < DEPTH_MAX_M)
    if ok.sum() < TABLE_STRIP_MIN_PTS:
        return None
    p_opt = np.stack([(uu[ok] - K.cx) * z[ok] / K.fx, (vv[ok] - K.cy) * z[ok] / K.fy, z[ok]], axis=1)
    zb = (p_opt @ extrinsics.R_base_opt.T + extrinsics.t_base_opt)[:, 2]
    if np.percentile(zb, 90) - np.percentile(zb, 10) > TABLE_STRIP_MAX_SPREAD_M:
        return None  # not a flat surface under the box (edge of the table, clutter)
    return float(np.median(zb))


def radius_from_silhouette(p_surface_base: np.ndarray, bbox: tuple[int, int, int, int], K: CameraIntrinsics,
                           extrinsics: Extrinsics, z_table: float, z_top: float) -> float | None:
    """Cylinder radius such that its silhouette is as wide as the box.

    The silhouette width is the extent in u of the top and bottom circles;
    the axis is the surface sample pushed horizontally by r, so the width is
    a monotone function of r, solved by bisection within RADIUS_RANGE_M.
    """
    x1, y1, x2, y2 = bbox
    w_box = float(x2 - x1)
    cam = extrinsics.t_base_opt
    ray = p_surface_base - cam
    horiz = np.array([ray[0], ray[1], 0.0])
    if np.linalg.norm(horiz) < 1e-6:
        return None
    horiz /= np.linalg.norm(horiz)

    def width(r: float) -> float:
        cx, cy = (p_surface_base + r * horiz)[:2]
        pts = np.vstack([_circle(cx, cy, z_table, r), _circle(cx, cy, z_top, r)])
        p_opt = (pts - cam) @ extrinsics.R_base_opt
        z = p_opt[:, 2]
        u = K.cx + p_opt[:, 0] * K.fx / np.where(z > 1e-6, z, np.nan)
        return float(np.nanmax(u) - np.nanmin(u))

    lo, hi = RADIUS_RANGE_M
    if not (width(lo) <= w_box <= width(hi)):
        return None
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        if width(mid) < w_box:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-4:
            break
    return 0.5 * (lo + hi)


def top_height_in_box(depth_m: np.ndarray, bbox: tuple[int, int, int, int], K: CameraIntrinsics,
                      extrinsics: Extrinsics, depth_front_m: float, radius_m: float, z_table: float) -> float | None:
    """Base-frame z of the cylinder's top, from the depth points inside the box.

    Looks at the central column band of the box, keeps the points that belong
    to the can (above the table, not farther than the can's back) and takes a
    high percentile of their height. None when too few points.
    """
    x1, y1, x2, y2 = bbox
    h, w = depth_m.shape[:2]
    uc = (x1 + x2) // 2
    u0, u1 = max(0, uc - TABLE_STRIP_HALF_W), min(w, uc + TABLE_STRIP_HALF_W + 1)
    v0, v1 = max(0, y1), min(h, y2 + 1)
    if v1 - v0 < 3 or u1 - u0 < 3:
        return None
    vv, uu = np.mgrid[v0:v1, u0:u1]
    z = depth_m[v0:v1, u0:u1]
    ok = (z > DEPTH_MIN_M) & (z < depth_front_m + 2.0 * radius_m + 0.05)
    if ok.sum() < TABLE_STRIP_MIN_PTS:
        return None
    p_opt = np.stack([(uu[ok] - K.cx) * z[ok] / K.fx, (vv[ok] - K.cy) * z[ok] / K.fy, z[ok]], axis=1)
    zb = (p_opt @ extrinsics.R_base_opt.T + extrinsics.t_base_opt)[:, 2]
    rows = vv[ok]
    keep = zb > z_table + TOP_MIN_ABOVE_TABLE_M
    zb, rows = zb[keep], rows[keep]
    if zb.size < TABLE_STRIP_MIN_PTS:
        return None
    z_top = float(np.percentile(zb, TOP_PERCENTILE))
    # The top must actually be seen: the points forming it have to reach the
    # upper part of the box, otherwise the depth on top is missing and this
    # would be the height of whatever part of the can has depth.
    top_rows = rows[zb >= z_top - 0.01]
    if top_rows.size == 0 or top_rows.min() > y1 + TOP_OBSERVED_FRAC * (y2 - y1):
        return None
    return z_top


def upright_cylinder_center(
    p_surface_base: np.ndarray,
    bbox: tuple[int, int, int, int],
    K: CameraIntrinsics,
    extrinsics: Extrinsics,
    radius_m: float,
    height_m: float,
    image_height: int,
    z_table: float | None = None,
) -> np.ndarray | None:
    """Centre of an upright cylinder from one front-surface sample and its box.

    ``z_table``, when given (from table_height_below_box), is used directly
    for the height; otherwise it is solved from the box's bottom row.

    ``p_surface_base`` is the back-projected depth sample at the box centre
    (a point on the cylinder's front surface, at whatever height that pixel
    hit). Assumes the base frame is z-up and the cylinder stands on a
    horizontal surface:

    * x, y: the surface point pushed by ``radius_m`` along the HORIZONTAL
      projection of the view ray (the spec's push along the 3-D ray is only
      right for a sphere; from a camera looking down it also drags the point
      down by radius * sin(pitch)).
    * z: the box's bottom row is where the cylinder's bottom circle (its
      contact line with the table) projects lowest in the image. With x, y
      known, the table height is the z at which that circle's lowest
      projected row equals the box's bottom row: a monotone 1-D problem,
      solved by bisection. The centre is half the height above it.

    Returns None when the geometry is unusable (box clipped at the bottom of
    the image, or a near-vertical view ray), so the caller can fall back.
    """
    x1, y1, x2, y2 = bbox
    if y2 >= image_height - 1:
        return None  # bottom edge not visible: contact line unknown
    cam = extrinsics.t_base_opt
    ray = p_surface_base - cam
    horiz = np.array([ray[0], ray[1], 0.0])
    h_norm = np.linalg.norm(horiz)
    if h_norm < MIN_HORIZONTAL_RAY * max(np.linalg.norm(ray), 1e-9):
        return None
    horiz /= h_norm
    cx, cy = (p_surface_base + radius_m * horiz)[:2]
    if z_table is not None:
        # Centre the axis on the silhouette: shift it across the view
        # direction until the projected silhouette's mid column equals the
        # box's. A sample that is not exactly facing the camera otherwise
        # leaves the axis a few mm to the side.
        across = np.array([-horiz[1], horiz[0], 0.0])
        u_box = 0.5 * (x1 + x2)
        for _ in range(3):
            pts = np.vstack([_circle(cx, cy, z_table, radius_m), _circle(cx, cy, z_table + height_m, radius_m)])
            p_opt = (pts - cam) @ extrinsics.R_base_opt
            z = p_opt[:, 2]
            if not (z > 1e-6).all():
                break
            u = K.cx + p_opt[:, 0] * K.fx / z
            du = u_box - 0.5 * (u.min() + u.max())
            z_axis = float(((np.array([cx, cy, z_table + height_m / 2.0]) - cam) @ extrinsics.R_base_opt)[2])
            shift = du * z_axis / K.fx * across  # image +u is optical +x; sign checked below
            sign = 1.0 if (extrinsics.R_base_opt.T @ across)[0] > 0 else -1.0
            cx, cy = cx + sign * shift[0], cy + sign * shift[1]
            if abs(du) < 0.1:
                break
        return np.array([cx, cy, z_table + height_m / 2.0])

    def lowest_row(z_table: float) -> float:
        rows = project_rows(_circle(cx, cy, z_table, radius_m), K, extrinsics)
        return float(np.nanmax(rows)) if np.isfinite(rows).any() else math.nan

    # The sample sits somewhere on the can, so the table is within one can
    # height below it. Lower table -> larger row, so f is decreasing in z.
    lo, hi = p_surface_base[2] - height_m - 0.02, p_surface_base[2] + 0.02
    f_lo, f_hi = lowest_row(lo) - y2, lowest_row(hi) - y2
    if not (math.isfinite(f_lo) and math.isfinite(f_hi)) or f_lo < 0 or f_hi > 0:
        return None  # bottom row not bracketed: box inconsistent with the model
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if lowest_row(mid) - y2 > 0:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-4:
            break
    z_table = 0.5 * (lo + hi)
    return np.array([cx, cy, z_table + height_m / 2.0])


@dataclass
class Detection:
    label: str
    score: float
    #: object centre in BASE_FRAME; None only when run_detection_pipeline was
    #: asked to keep detections whose depth was missing/out of range
    p_base: np.ndarray | None
    bbox: tuple[int, int, int, int] = (0, 0, 0, 0)  # x1, y1, x2, y2 (px)
    uv: tuple[int, int] = (0, 0)  # bbox centre (px), where depth was sampled
    depth_m: float | None = None  # surface depth at uv, before the radius push
    height_m: float | None = None  # measured cylinder height (None = fallback constant used)
    radius_m: float | None = None  # measured cylinder radius (None = fallback constant used)


def run_detection_pipeline(
    rgb: np.ndarray,
    depth_mm: np.ndarray,
    model: YOLO,
    K: CameraIntrinsics,
    extrinsics: Extrinsics,
    conf: float = 0.40,
    iou: float = 0.45,
    include_invalid_depth: bool = False,
) -> list[Detection]:
    """Steps 1-5 of the spec's algorithm, run once on one RGB/depth pair.

    Detections whose depth sample is missing or outside [DEPTH_MIN_M,
    DEPTH_MAX_M] are dropped, unless ``include_invalid_depth`` is set, in which
    case they are returned with ``p_base=None`` (for visualisation only).
    """
    depth_m = depth_mm.astype(np.float64) * 0.001
    results = model.predict(rgb, conf=conf, iou=iou, verbose=False)[0]

    records: list[Detection] = []
    for box in results.boxes:
        label = results.names[int(box.cls.item())]
        score = float(box.conf.item())
        xyxy = box.xyxy[0].cpu().numpy()
        bbox = tuple(int(round(float(c))) for c in xyxy)
        if label in CLASS_HEIGHT_M:  # front face, below the visible top
            u, v = cylinder_sample_pixel(bbox)
            z_c = median_depth_in_box(depth_m, bbox, rows=CYL_SAMPLE_ROWS)
        else:
            u, v = bbox_center(xyxy)
            z_c = median_depth_in_box(depth_m, bbox)
        if z_c is None or not (DEPTH_MIN_M <= z_c <= DEPTH_MAX_M):
            if include_invalid_depth:
                records.append(Detection(label=label, score=score, p_base=None, bbox=bbox, uv=(u, v), depth_m=z_c))
            continue

        p_optical = backproject_pixel(u, v, z_c, K)
        radius = CLASS_RADIUS_M.get(label, DEFAULT_RADIUS_M)
        height = CLASS_HEIGHT_M.get(label)
        p_base, meas_h, meas_r = None, None, None
        if height is not None:  # upright cylinder: measure it
            p_surface = optical_to_base(p_optical, extrinsics)
            z_table = table_height_below_box(depth_m, bbox, K, extrinsics)
            # A box touching the image border is cut off: its top row is not
            # the top of the can and its width is not the silhouette.
            clipped = bbox[0] <= 0 or bbox[1] <= 0 or bbox[2] >= depth_m.shape[1] - 1 or bbox[3] >= depth_m.shape[0] - 1
            if z_table is not None and not clipped:
                z_top = top_height_in_box(depth_m, bbox, K, extrinsics, z_c, radius, z_table)
                if z_top is not None and HEIGHT_RANGE_M[0] <= z_top - z_table <= HEIGHT_RANGE_M[1]:
                    height, meas_h = z_top - z_table, z_top - z_table
                r_est = radius_from_silhouette(p_surface, bbox, K, extrinsics, z_table, z_table + height)
                if r_est is not None:
                    radius, meas_r = r_est, r_est
            p_base = upright_cylinder_center(
                optical_to_base(p_optical, extrinsics), bbox, K, extrinsics, radius, height, depth_m.shape[0],
                z_table=z_table)
        if p_base is None:  # spec's plain push along the view ray
            p_base = optical_to_base(push_along_ray(p_optical, radius), extrinsics)

        records.append(Detection(label=label, score=score, p_base=p_base, bbox=bbox, uv=(u, v), depth_m=z_c,
                                 height_m=meas_h, radius_m=meas_r))

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

    if index < 0:
        return ObjectPositionResult(success=False, message=f"index must be >= 0, got {index}")
    if index >= len(matches):
        return ObjectPositionResult(success=False, message=f"no match for label={label!r} at index={index}")

    p = matches[index].p_base
    return ObjectPositionResult(success=True, x=float(p[0]), y=float(p[1]), z=float(p[2]), frame_id=frame_id)
