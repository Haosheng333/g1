"""
02_train_mlp.py
================
Stage 2 of the IK pipeline: train ResidualMLPIK to map
(x, y, z, vx, vy) -> (q1..q5), then validate it not just in joint-angle
MSE, but in *physical Cartesian error* -- how far off (in cm) the
predicted joints actually put the end-effector, according to forward
kinematics. See the docstring on `evaluate_cartesian_error` for why this
second metric is the one that actually matters.
"""

import argparse
import time

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader, random_split

import arm_config as cfg
from model import ResidualMLPIK, denormalize_joints, normalize_joints


def build_dataloaders(dataset_path: str, batch_size: int, val_fraction: float, seed: int):
    data = torch.load(dataset_path)

    inputs = data["inputs"]                 # (N, 5) raw (x,y,z,vx,vy)
    targets = data["targets"]                # (N, 5) raw joint radians
    input_mean = data["input_mean"]
    input_std = data["input_std"]
    joint_lo = data["joint_lo"]
    joint_hi = data["joint_hi"]

    # Standardize inputs (z-score) and normalize targets to [-1, 1] using
    # the SAME stats that were computed and saved in stage 1 -- never
    # recomputed here, so train/val and any future inference all agree.
    inputs_norm = (inputs - input_mean) / input_std
    targets_norm = normalize_joints(targets, joint_lo, joint_hi)

    full_dataset = TensorDataset(inputs_norm, targets_norm, inputs, targets)

    n_val = int(len(full_dataset) * val_fraction)
    n_train = len(full_dataset) - n_val
    generator = torch.Generator().manual_seed(seed)
    train_set, val_set = random_split(full_dataset, [n_train, n_val], generator=generator)

    train_loader = DataLoader(train_set, batch_size=batch_size, shuffle=True, drop_last=True)
    val_loader = DataLoader(val_set, batch_size=batch_size, shuffle=False)

    return train_loader, val_loader, data


@torch.no_grad()
def evaluate_cartesian_error(model, val_loader, joint_lo, joint_hi, device):
    """
    Custom evaluation loop: physical Cartesian position error, in cm.

    Why not just report joint-angle MSE? Because MSE in *joint space* and
    error in *task space* (where the robot actually has to be accurate)
    are only loosely related, for a nonlinear kinematic chain:

      - Near an outstretched (near-singular) arm configuration, a tiny
        joint-angle error can translate into a large end-effector
        position error (the Jacobian is ill-conditioned there).
      - Conversely, some joint-angle error can be "absorbed" by the
        kinematic redundancy of a multi-joint chain and barely move the
        end-effector at all.

    So a model with excellent joint-space MSE could still miss the can by
    several centimeters, or vice versa. The only metric that actually
    answers "will the gripper reach the can" is: take the predicted
    joints, run them through the *same* forward kinematics used to build
    the dataset, and measure the Euclidean distance between that
    predicted position and the position we originally asked for. That is
    exactly what this loop does, using arm_config.fk_batch (the same
    function stage 1 used), so the metric is measured in the one space
    that matters for the demo: physical centimeters at the hand.
    """
    model.eval()
    all_errors_cm = []

    for inputs_norm, _, inputs_raw, _ in val_loader:
        inputs_norm = inputs_norm.to(device)
        pred_norm = model(inputs_norm)
        pred_joints = denormalize_joints(pred_norm.cpu(), joint_lo, joint_hi)

        pred_pos, _ = cfg.positions_and_approach(pred_joints.numpy())
        target_pos = inputs_raw[:, :3].numpy()  # the (x,y,z) we asked the model to reach

        err_m = np.linalg.norm(pred_pos - target_pos, axis=1)
        all_errors_cm.append(err_m * 100.0)

    all_errors_cm = np.concatenate(all_errors_cm)
    return {
        "mean_cm": float(all_errors_cm.mean()),
        "median_cm": float(np.median(all_errors_cm)),
        "p95_cm": float(np.percentile(all_errors_cm, 95)),
        "max_cm": float(all_errors_cm.max()),
    }


def main():
    parser = argparse.ArgumentParser(description="Train the IK MLP.")
    parser.add_argument("--dataset", type=str, default="ik_dataset.pt")
    parser.add_argument("--out", type=str, default="ik_model.pt")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=1024)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--val-fraction", type=float, default=0.10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--eval-every", type=int, default=10,
                         help="Run the Cartesian-error eval loop every N epochs.")
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    train_loader, val_loader, data = build_dataloaders(
        args.dataset, args.batch_size, args.val_fraction, args.seed
    )
    joint_lo, joint_hi = data["joint_lo"], data["joint_hi"]

    model = ResidualMLPIK().to(device)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Model: {cfg.NUM_RESIDUAL_BLOCKS} residual blocks x {cfg.HIDDEN_DIM} units, "
          f"{n_params:,} parameters")

    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)
    criterion = nn.MSELoss()

    best_mean_cm = float("inf")
    t0 = time.time()

    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        for inputs_norm, targets_norm, _, _ in train_loader:
            inputs_norm = inputs_norm.to(device)
            targets_norm = targets_norm.to(device)

            optimizer.zero_grad()
            pred_norm = model(inputs_norm)
            loss = criterion(pred_norm, targets_norm)
            loss.backward()
            optimizer.step()

            running_loss += loss.item() * inputs_norm.size(0)

        scheduler.step()
        train_loss = running_loss / len(train_loader.dataset)

        do_eval = (epoch % args.eval_every == 0) or (epoch == args.epochs)
        if do_eval:
            metrics = evaluate_cartesian_error(model, val_loader, joint_lo, joint_hi, device)
            elapsed = time.time() - t0
            print(f"Epoch {epoch:4d}/{args.epochs}  "
                  f"train_MSE={train_loss:.6f}  lr={scheduler.get_last_lr()[0]:.2e}  "
                  f"val_cartesian_err: mean={metrics['mean_cm']:.3f}cm "
                  f"median={metrics['median_cm']:.3f}cm p95={metrics['p95_cm']:.3f}cm "
                  f"max={metrics['max_cm']:.3f}cm  ({elapsed:.1f}s elapsed)")

            if metrics["mean_cm"] < best_mean_cm:
                best_mean_cm = metrics["mean_cm"]
                checkpoint = {
                    "model_state_dict": model.state_dict(),
                    "input_mean": data["input_mean"],
                    "input_std": data["input_std"],
                    "joint_lo": data["joint_lo"],
                    "joint_hi": data["joint_hi"],
                    "joint_names": data["joint_names"],
                    "workspace_bounds": data["workspace_bounds"],
                    "hidden_dim": cfg.HIDDEN_DIM,
                    "num_blocks": cfg.NUM_RESIDUAL_BLOCKS,
                    "epoch": epoch,
                    "val_cartesian_error_cm": metrics,
                }
                torch.save(checkpoint, args.out)
        else:
            print(f"Epoch {epoch:4d}/{args.epochs}  train_MSE={train_loss:.6f}  "
                  f"lr={scheduler.get_last_lr()[0]:.2e}")

    print(f"\nBest validation mean Cartesian error: {best_mean_cm:.3f} cm")
    print(f"Best checkpoint saved to: {args.out}")


if __name__ == "__main__":
    main()
