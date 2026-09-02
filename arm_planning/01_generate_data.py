"""
01_generate_data.py
====================
Stage 1 of the IK pipeline: build a supervised dataset of

    (x, y, z, vx, vy)  ->  (q1, q2, q3, q4, q5)

by sampling random, physically valid joint configurations of the Unitree
G1 right arm, running them through forward kinematics, and keeping only
the samples whose end-effector lands inside the front "reach for a can"
workspace box defined in arm_config.py.

Why sample joints and run FK forward, instead of sampling Cartesian points
directly? Because forward kinematics is a well-defined function (joints ->
pose) while inverse kinematics is what we're trying to teach the network,
and is generally one-to-many (multiple joint configs can reach the same
pose). Sampling in joint space guarantees every generated example is
reachable and physically valid by construction -- no wasted/impossible
targets, and we get the training pairs for free.

Performance note
-----------------
Calling yourdfpy once per sample to reach 1,000,000 valid poses would take
on the order of tens of minutes (each call re-walks the whole 23-DOF
kinematic tree in pure Python). Instead we use arm_config.fk_batch, a
vectorized numpy implementation of just the 5-DOF arm chain, verified
against yourdfpy to ~1e-15 agreement (see project notes). This lets us
forward-kinematics an entire batch of candidates in one shot with numpy's
batched matrix multiply, so generating a million filtered samples takes
seconds, not minutes.
"""

import argparse
import time

import numpy as np
import torch

import arm_config as cfg


def sample_uniform_joint_angles(n: int, rng: np.random.Generator) -> np.ndarray:
    """Uniform random joint angles within each joint's URDF limits, shape (n, 5)."""
    lo = cfg.JOINT_LIMITS[:, 0]
    hi = cfg.JOINT_LIMITS[:, 1]
    return rng.uniform(lo, hi, size=(n, cfg.NUM_JOINTS))


def generate_dataset(target_valid: int, batch_size: int, seed: int, verbose: bool = True):
    rng = np.random.default_rng(seed)

    all_joints = []
    all_pos = []
    all_approach = []

    n_valid = 0
    n_tried = 0
    t0 = time.time()

    while n_valid < target_valid:
        candidates = sample_uniform_joint_angles(batch_size, rng)
        pos, approach2d = cfg.positions_and_approach(candidates)
        mask = cfg.in_workspace(pos)

        n_tried += batch_size
        n_hit = int(mask.sum())
        if n_hit == 0:
            continue

        all_joints.append(candidates[mask])
        all_pos.append(pos[mask])
        all_approach.append(approach2d[mask])
        n_valid += n_hit

        if verbose and (len(all_joints) % 20 == 0 or n_valid >= target_valid):
            rate = n_valid / max(n_tried, 1)
            elapsed = time.time() - t0
            print(f"  valid={n_valid:>9,}/{target_valid:,}  "
                  f"acceptance={rate:6.2%}  elapsed={elapsed:5.1f}s")

    joints = np.concatenate(all_joints, axis=0)[:target_valid]
    pos = np.concatenate(all_pos, axis=0)[:target_valid]
    approach2d = np.concatenate(all_approach, axis=0)[:target_valid]

    elapsed = time.time() - t0
    print(f"Done: {target_valid:,} valid samples from {n_tried:,} candidates "
          f"({target_valid / max(n_tried,1):.2%} acceptance) in {elapsed:.1f}s")

    return joints, pos, approach2d


def main():
    parser = argparse.ArgumentParser(description="Generate IK training dataset.")
    parser.add_argument("--num-samples", type=int, default=1_000_000,
                         help="Target number of valid samples (default 1,000,000).")
    parser.add_argument("--batch-size", type=int, default=200_000,
                         help="Candidates per rejection-sampling batch.")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out", type=str, default="ik_dataset.pt")
    args = parser.parse_args()

    print(f"Sampling up to {args.num_samples:,} valid poses inside the front "
          f"workspace box:\n  {cfg.WORKSPACE_BOUNDS}")
    joints, pos, approach2d = generate_dataset(
        target_valid=args.num_samples,
        batch_size=args.batch_size,
        seed=args.seed,
    )

    # Cartesian input features: (x, y, z, vx, vy)
    inputs = np.concatenate([pos, approach2d], axis=1).astype(np.float32)
    targets = joints.astype(np.float32)

    # --- Normalization stats, computed here so stage 2 doesn't need to
    #     recompute (and can't accidentally compute them differently). ---
    # Inputs: standard (z-score) normalization -- learned from the data,
    # since the Cartesian workspace box has no a-priori symmetric range.
    input_mean = inputs.mean(axis=0)
    input_std = inputs.std(axis=0)
    input_std[input_std < 1e-8] = 1.0  # guard against degenerate dims

    # Targets: min-max normalization to [-1, 1] using the *physical* joint
    # limits from the URDF, not the empirical min/max of this dataset.
    # Using the true limits (a) guarantees the mapping is valid for any
    # future joint value in range, not just ones this random sample
    # happened to draw, and (b) matches the network's tanh output head
    # exactly, so every point in [-1, 1] output space is a physically
    # realizable joint angle -- the network literally cannot predict an
    # out-of-limits joint command.
    joint_lo = cfg.JOINT_LIMITS[:, 0]
    joint_hi = cfg.JOINT_LIMITS[:, 1]

    dataset = {
        "inputs": torch.from_numpy(inputs),                       # (N, 5) float32
        "targets": torch.from_numpy(targets),                     # (N, 5) float32
        "input_mean": torch.from_numpy(input_mean.astype(np.float32)),
        "input_std": torch.from_numpy(input_std.astype(np.float32)),
        "joint_lo": torch.from_numpy(joint_lo.astype(np.float32)),
        "joint_hi": torch.from_numpy(joint_hi.astype(np.float32)),
        "joint_names": cfg.JOINT_NAMES,
        "workspace_bounds": cfg.WORKSPACE_BOUNDS,
        "base_link": cfg.BASE_LINK,
        "ee_link": cfg.EE_LINK,
        "urdf_path": cfg.URDF_PATH,
    }

    torch.save(dataset, args.out)
    print(f"Saved {inputs.shape[0]:,} samples -> {args.out}")
    print(f"Input feature ranges (x,y,z,vx,vy):")
    print(f"  min: {inputs.min(axis=0)}")
    print(f"  max: {inputs.max(axis=0)}")


if __name__ == "__main__":
    main()
