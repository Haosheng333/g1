"""
03_plan_path.py
================
Stage 3 of the IK pipeline: given a start pose and an end pose (each a
Cartesian (x, y, z, vx, vy) target -- e.g. "hover above the can" ->
"grasp position at the can"), generate a smooth 20-step linear Cartesian
path between them and use the trained network to infer the joint-space
trajectory the arm should follow to trace it.

This is the "closes the loop" script: a CV system would in principle
supply the (x, y, z) of the detected can and a desired approach direction;
this script shows how that single target (or a pair of targets, for a
pre-grasp hover pose -> final grasp pose) becomes a full joint trajectory
ready to send to the robot's controllers, at inference speed (a single
forward pass per waypoint, no iterative IK solver in the loop).
"""

import argparse

import numpy as np
import torch

import arm_config as cfg
from model import ResidualMLPIK, denormalize_joints


def load_model(checkpoint_path: str, device: torch.device):
    checkpoint = torch.load(checkpoint_path, map_location=device)

    # Reconstruct the EXACT architecture used at training time, from the
    # hyperparameters saved in the checkpoint -- never re-guessed here.
    model = ResidualMLPIK(
        input_dim=cfg.INPUT_DIM,
        hidden_dim=checkpoint["hidden_dim"],
        output_dim=cfg.OUTPUT_DIM,
        num_blocks=checkpoint["num_blocks"],
    ).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    return model, checkpoint


def make_linear_cartesian_path(start_pose: np.ndarray, end_pose: np.ndarray, n_steps: int) -> np.ndarray:
    """
    Linearly interpolate between two (x, y, z, vx, vy) poses in n_steps
    waypoints (inclusive of both endpoints).

    Position (x, y, z) is interpolated directly -- a straight-line
    Cartesian path is exactly what "linear interpolation of the position"
    means, and is the standard choice for a short reach-and-grab motion.

    The approach vector (vx, vy) is a *direction*, not a free point, so
    linearly interpolating its two components and then re-normalizing to
    unit length approximates a spherical (slerp) interpolation of the
    approach direction. For the small angular sweeps typical of a
    reach-to-grasp motion this linear+renormalize approximation is very
    close to true slerp; for large direction changes you'd want to
    interpolate the angle directly instead.
    """
    t = np.linspace(0.0, 1.0, n_steps)[:, None]  # (n_steps, 1)
    pos = start_pose[None, :3] * (1 - t) + end_pose[None, :3] * t
    approach = start_pose[None, 3:5] * (1 - t) + end_pose[None, 3:5] * t
    approach = approach / np.clip(np.linalg.norm(approach, axis=1, keepdims=True), 1e-8, None)
    return np.concatenate([pos, approach], axis=1).astype(np.float32)


@torch.no_grad()
def infer_joint_trajectory(model, checkpoint, cartesian_path: np.ndarray, device: torch.device):
    input_mean = checkpoint["input_mean"]
    input_std = checkpoint["input_std"]
    joint_lo = checkpoint["joint_lo"]
    joint_hi = checkpoint["joint_hi"]

    inputs = torch.from_numpy(cartesian_path)
    inputs_norm = (inputs - input_mean) / input_std
    inputs_norm = inputs_norm.to(device)

    pred_norm = model(inputs_norm).cpu()
    pred_joints = denormalize_joints(pred_norm, joint_lo, joint_hi)
    return pred_joints.numpy()


def main():
    parser = argparse.ArgumentParser(description="Plan a Cartesian path and infer the joint trajectory.")
    parser.add_argument("--checkpoint", type=str, default="ik_model.pt")
    parser.add_argument("--steps", type=int, default=20)
    # Defaults: a pre-grasp hover pose above/behind the can, moving to a
    # grasp pose closer to the can -- both inside WORKSPACE_BOUNDS.
    parser.add_argument("--start", type=float, nargs=5, default=[0.18, -0.10, 0.15, 1.0, 0.0],
                         metavar=("X", "Y", "Z", "VX", "VY"))
    parser.add_argument("--end", type=float, nargs=5, default=[0.30, -0.20, 0.00, 1.0, 0.0],
                         metavar=("X", "Y", "Z", "VX", "VY"))
    parser.add_argument("--out", type=str, default="joint_trajectory.csv")
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model, checkpoint = load_model(args.checkpoint, device)
    print(f"Loaded checkpoint from epoch {checkpoint['epoch']} "
          f"(val Cartesian error: {checkpoint['val_cartesian_error_cm']['mean_cm']:.3f} cm mean)")

    start_pose = np.array(args.start, dtype=np.float32)
    end_pose = np.array(args.end, dtype=np.float32)

    cartesian_path = make_linear_cartesian_path(start_pose, end_pose, args.steps)
    joint_trajectory = infer_joint_trajectory(model, checkpoint, cartesian_path, device)

    # Validate the plan: run the predicted joints back through forward
    # kinematics and report how far each waypoint actually lands from
    # where we asked it to go (same physical-error philosophy as stage 2).
    achieved_pos, _ = cfg.positions_and_approach(joint_trajectory)
    target_pos = cartesian_path[:, :3]
    err_cm = np.linalg.norm(achieved_pos - target_pos, axis=1) * 100.0

    print(f"\n{'step':>4}  {'target (x,y,z) m':<26}  " +
          "  ".join(f"q{i+1}(deg)" for i in range(cfg.NUM_JOINTS)) +
          f"  {'err(cm)':>8}")
    for i in range(args.steps):
        tgt = target_pos[i]
        q_deg = np.degrees(joint_trajectory[i])
        q_str = "  ".join(f"{q:7.2f}" for q in q_deg)
        print(f"{i:4d}  ({tgt[0]:.3f},{tgt[1]:.3f},{tgt[2]:.3f})      {q_str}  {err_cm[i]:8.3f}")

    print(f"\nMean tracking error along path: {err_cm.mean():.3f} cm "
          f"(max {err_cm.max():.3f} cm)")

    header = "x,y,z,vx,vy," + ",".join(cfg.JOINT_NAMES) + ",cartesian_err_cm"
    rows = np.concatenate([cartesian_path, joint_trajectory, err_cm[:, None]], axis=1)
    np.savetxt(args.out, rows, delimiter=",", header=header, comments="")
    print(f"Saved joint trajectory -> {args.out}")


if __name__ == "__main__":
    main()
