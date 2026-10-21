"""Draw what perception sees onto the camera image (ROS-free, needs cv2).

Used by the viewer node (live window + /perception/annotated_image) and by
demo_m1.py. Green = the label being looked for, yellow = other classes,
red = a box whose depth sample was missing/out of range (no position).
"""

from __future__ import annotations

import cv2
import numpy as np

from perception.m1_perception import BASE_FRAME, Detection

GREEN = (0, 200, 0)
YELLOW = (0, 200, 255)
RED = (0, 0, 255)
WHITE = (255, 255, 255)


def _put(img, text, org, color, scale=0.55):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_detections(bgr: np.ndarray, detections: list[Detection], target_label: str = "",
                    header: str = "") -> np.ndarray:
    """Return a copy of ``bgr`` with boxes, labels, depth and base-frame xyz drawn."""
    out = bgr.copy()
    for d in detections:
        x1, y1, x2, y2 = d.bbox
        if d.p_base is None:
            color = RED
        elif target_label and d.label == target_label:
            color = GREEN
        else:
            color = YELLOW if target_label else GREEN
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cv2.drawMarker(out, d.uv, color, cv2.MARKER_CROSS, 12, 2)
        depth = "depth n/a" if d.depth_m is None else f"d={d.depth_m:.2f}m"
        size = ""
        if d.height_m is not None:
            size = f"  h={d.height_m * 100:.1f}cm" + (f" r={d.radius_m * 100:.1f}cm" if d.radius_m is not None else "")
        elif d.p_base is not None and d.label in __import__("perception.m1_perception", fromlist=["CLASS_HEIGHT_M"]).CLASS_HEIGHT_M:
            size = "  h=? (fallback)"
        _put(out, f"{d.label} {d.score:.2f}  {depth}{size}", (x1, max(14, y1 - 6)), color)
        if d.p_base is not None:
            _put(out, f"{BASE_FRAME} ({d.p_base[0]:+.2f}, {d.p_base[1]:+.2f}, {d.p_base[2]:+.2f}) m",
                 (x1, min(out.shape[0] - 4, y2 + 18)), color)
    if header:
        band = out[:30]
        band[:] = (band * 0.35).astype(np.uint8)  # dark strip so the header stays readable
        _put(out, header, (8, 21), WHITE, 0.55)
    return out


def depth_to_bgr(depth_mm: np.ndarray, max_m: float = 3.0) -> np.ndarray:
    """Colour-map a 16UC1 depth image; 0 (no return) is drawn black."""
    d = depth_mm.astype(np.float32) * 0.001
    scaled = np.clip(d / max_m, 0.0, 1.0) * 255.0
    colour = cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_JET)
    colour[depth_mm == 0] = 0
    return colour


def side_by_side(bgr: np.ndarray, depth_mm: np.ndarray, detections: list[Detection], max_m: float = 3.0) -> np.ndarray:
    """Annotated colour on the left, depth with the same boxes on the right."""
    depth_vis = depth_to_bgr(depth_mm, max_m)
    for d in detections:
        x1, y1, x2, y2 = d.bbox
        cv2.rectangle(depth_vis, (x1, y1), (x2, y2), WHITE if d.p_base is not None else RED, 1)
        cv2.drawMarker(depth_vis, d.uv, WHITE, cv2.MARKER_CROSS, 10, 1)
    if depth_vis.shape[:2] != bgr.shape[:2]:
        depth_vis = cv2.resize(depth_vis, (bgr.shape[1], bgr.shape[0]))
    return np.hstack([bgr, depth_vis])
