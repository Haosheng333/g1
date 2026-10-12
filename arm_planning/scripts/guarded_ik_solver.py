#!/usr/bin/env python3
"""Runtime IK inference with mandatory FK verification before accepting joints."""
from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
import argparse
import json
import math
import sys

import numpy as np
import torch
import yaml
from torch import nn

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from g1_forward_kinematics import pose_5d


class HybridIKNet(nn.Module):
    """Must match the architecture used by train_ik_hybrid.py."""
    def __init__(self, lower, upper):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(5, 256), nn.ReLU(),
            nn.Linear(256, 512), nn.ReLU(),
            nn.Linear(512, 256), nn.ReLU(),
            nn.Linear(256, 5),
        )
        lower = torch.as_tensor(lower, dtype=torch.float32)
        upper = torch.as_tensor(upper, dtype=torch.float32)
        self.register_buffer("joint_midpoint", (lower + upper) / 2.0)
        self.register_buffer("joint_half_range", (upper - lower) / 2.0)

    def forward(self, x):
        return self.joint_midpoint + self.joint_half_range * torch.tanh(self.net(x))


@dataclass
class IKVerificationResult:
    success: bool
    arm: str
    requested_pose: list[float]
    reconstructed_pose: list[float] | None
    joints_rad: list[float] | None
    position_error_m: float | None
    rx_error_rad: float | None
    rz_error_rad: float | None
    message: str


def wrapped_difference(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose_in_workspace(pose, config) -> bool:
    ws = config.get("training_workspace", {})
    if not ws.get("enabled", False):
        return True
    x, y, z, _, _ = pose
    return (
        ws["x_min"] <= x <= ws["x_max"]
        and ws["y_min"] <= y <= ws["y_max"]
        and ws["z_min"] <= z <= ws["z_max"]
    )


class GuardedIKSolver:
    def __init__(
        self,
        checkpoint_path: str | Path,
        arm: str,
        config_path: str | Path,
        max_position_error_m: float = 0.01,
        max_rx_error_rad: float = 0.10,
        max_rz_error_rad: float = 0.15,
        device: str = "cpu",
    ):
        self.arm = arm
        self.device = torch.device(device)
        self.max_position_error_m = float(max_position_error_m)
        self.max_rx_error_rad = float(max_rx_error_rad)
        self.max_rz_error_rad = float(max_rz_error_rad)

        with Path(config_path).open("r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        if arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")

        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        checkpoint_arm = checkpoint.get("arm")
        if checkpoint_arm is not None and checkpoint_arm != arm:
            raise ValueError(f"Checkpoint arm is {checkpoint_arm!r}, requested {arm!r}")

        lower = np.asarray(self.config[arm]["lower_limits"], dtype=np.float32)
        upper = np.asarray(self.config[arm]["upper_limits"], dtype=np.float32)
        self.model = HybridIKNet(lower, upper).to(self.device)
        self.model.load_state_dict(checkpoint["model_state"])
        self.model.eval()

        self.x_mean = np.asarray(checkpoint["x_mean"], dtype=np.float32)
        self.x_std = np.asarray(checkpoint["x_std"], dtype=np.float32)
        if np.any(self.x_std <= 0) or self.x_mean.shape != (5,) or self.x_std.shape != (5,):
            raise ValueError("Checkpoint contains invalid input normalization statistics")

        self.lower = lower
        self.upper = upper

    def solve(self, requested_pose) -> IKVerificationResult:
        pose = np.asarray(requested_pose, dtype=np.float64)
        if pose.shape != (5,) or not np.all(np.isfinite(pose)):
            return IKVerificationResult(False, self.arm, pose.tolist() if pose.ndim == 1 else [], None, None, None, None, None, "Requested pose must contain five finite values [x,y,z,rx,rz]")

        if not pose_in_workspace(pose, self.config):
            return IKVerificationResult(False, self.arm, pose.tolist(), None, None, None, None, None, "Requested pose is outside the configured training workspace")

        scaled = ((pose.astype(np.float32) - self.x_mean) / self.x_std)[None, :]
        with torch.no_grad():
            joints = self.model(torch.from_numpy(scaled).to(self.device)).cpu().numpy()[0]

        # Numerical defence even though tanh already constrains the model output.
        joints = np.clip(joints, self.lower, self.upper)
        reconstructed = pose_5d(joints.astype(np.float64), self.arm, self.config)

        position_error = float(np.linalg.norm(reconstructed[:3] - pose[:3]))
        rx_error = abs(wrapped_difference(float(reconstructed[3]), float(pose[3])))
        rz_error = abs(wrapped_difference(float(reconstructed[4]), float(pose[4])))

        failures = []
        if position_error > self.max_position_error_m:
            failures.append(f"position error {position_error:.6f} m exceeds {self.max_position_error_m:.6f} m")
        if rx_error > self.max_rx_error_rad:
            failures.append(f"rx error {rx_error:.6f} rad exceeds {self.max_rx_error_rad:.6f} rad")
        if rz_error > self.max_rz_error_rad:
            failures.append(f"rz error {rz_error:.6f} rad exceeds {self.max_rz_error_rad:.6f} rad")

        if failures:
            # Do not expose the rejected joints as an executable result.
            return IKVerificationResult(False, self.arm, pose.tolist(), reconstructed.tolist(), None, position_error, rx_error, rz_error, "; ".join(failures))

        return IKVerificationResult(True, self.arm, pose.tolist(), reconstructed.tolist(), joints.astype(float).tolist(), position_error, rx_error, rz_error, "IK solution passed runtime FK verification")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--arm", choices=["left", "right"], required=True)
    parser.add_argument("--config", type=Path, default=Path("config/arm_kinematics.yaml"))
    parser.add_argument("--pose", nargs=5, type=float, metavar=("X", "Y", "Z", "RX", "RZ"), required=True)
    parser.add_argument("--max-position-error", type=float, default=0.01)
    parser.add_argument("--max-rx-error", type=float, default=0.10)
    parser.add_argument("--max-rz-error", type=float, default=0.15)
    args = parser.parse_args()

    solver = GuardedIKSolver(
        args.checkpoint, args.arm, args.config,
        args.max_position_error, args.max_rx_error, args.max_rz_error,
    )
    result = solver.solve(args.pose)
    print(json.dumps(asdict(result), indent=2))
    raise SystemExit(0 if result.success else 2)


if __name__ == "__main__":
    main()
