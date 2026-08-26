"""Validation for perception.m1_perception.

No real RealSense is available on this laptop, so validation has two parts:
1. A synthetic round-trip test of the math (back-projection + ray-push +
   extrinsic transform) against a known 3D point -- this is the offline
   equivalent of the spec's "tape measure" acceptance test.
2. A run of the real pretrained YOLO detector against an actual photo
   (bundled in test/data/, a "remote" -- a COCO class) paired with a
   synthetic depth plane, to confirm the full pipeline executes end to end
   and returns a plausible point.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from perception.m1_perception import (
    CameraIntrinsics,
    Extrinsics,
    backproject_pixel,
    handle_get_object_position,
    median_nonzero_depth,
    optical_to_base,
    push_along_ray,
)

TEST_IMAGE = Path(__file__).parent / "data" / "sample_remote.png"


def test_backprojection_round_trip() -> None:
    K = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
    true_point_optical = np.array([0.12, -0.05, 1.40])  # meters, camera optical frame

    u = K.cx + true_point_optical[0] * K.fx / true_point_optical[2]
    v = K.cy + true_point_optical[1] * K.fy / true_point_optical[2]
    z = true_point_optical[2]

    recovered = backproject_pixel(int(round(u)), int(round(v)), z, K)
    error = np.linalg.norm(recovered - true_point_optical)
    assert error < 0.01, f"back-projection round trip error too large: {error:.4f} m"
    print(f"[ok] back-projection round trip error = {error * 100:.2f} cm")


def test_depth_patch_median_rejects_zeros() -> None:
    depth = np.zeros((480, 640), dtype=np.float64)
    depth[237:244, 317:324] = 1.4
    depth[240, 320] = 0.0  # a hole exactly at the bbox center
    z = median_nonzero_depth(depth, 320, 240)
    assert z is not None and abs(z - 1.4) < 1e-9
    print(f"[ok] patch-median depth sampling survives a center hole: z={z}")


def test_push_along_ray_and_extrinsics() -> None:
    p_optical = np.array([0.0, 0.0, 1.0])
    pushed = push_along_ray(p_optical, radius_m=0.05)
    assert abs(pushed[2] - 1.05) < 1e-9, "should push straight along +z for a point on the optical axis"

    # 90 deg rotation about z: optical-frame x becomes base-frame y
    R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)
    t = np.array([0.1, 0.0, 0.5])
    p_base = optical_to_base(np.array([1.0, 0.0, 0.0]), Extrinsics(R_base_opt=R, t_base_opt=t))
    assert np.allclose(p_base, [0.1, 1.0, 0.5], atol=1e-9)
    print("[ok] ray push + extrinsic transform match expected geometry")


def test_end_to_end_on_real_photo() -> None:
    import cv2
    from ultralytics import YOLO

    rgb = cv2.imread(str(TEST_IMAGE))
    assert rgb is not None, f"missing test image {TEST_IMAGE}"

    depth_mm = np.full(rgb.shape[:2], 1400, dtype=np.uint16)  # synthetic flat 1.4m plane

    model = YOLO("yolov8n.pt")
    K = CameraIntrinsics(fx=615.0, fy=615.0, cx=rgb.shape[1] / 2, cy=rgb.shape[0] / 2)
    # explicit identity (not the Extrinsics() default, which is now the real
    # G1+D435i mount): this test wants camera-optical-frame output, unrelated
    # to the sample photo's imaginary camera placement
    extrinsics = Extrinsics(R_base_opt=np.eye(3), t_base_opt=np.zeros(3))

    result = handle_get_object_position(rgb, depth_mm, model, K, extrinsics, label="remote", index=0)
    assert result.success, f"expected a 'remote' detection, got: {result.message}"
    assert 0.15 <= result.z <= 5.0
    print(f"[ok] end-to-end pipeline on real photo -> x={result.x:.3f} y={result.y:.3f} z={result.z:.3f}")


if __name__ == "__main__":
    test_backprojection_round_trip()
    test_depth_patch_median_rejects_zeros()
    test_push_along_ray_and_extrinsics()
    test_end_to_end_on_real_photo()
    print("\nAll Module 1 validation checks passed.")
