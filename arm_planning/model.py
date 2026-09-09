"""
model.py
========
The IK network architecture, defined once and imported by both
02_train_mlp.py (to train it) and 03_plan_path.py (to reconstruct the
identical module graph before loading trained weights). Keeping the class
in one place -- rather than copy-pasted into both scripts -- is what
actually guarantees "exact architecture reconstruction": there is only
one definition that can possibly exist.
"""

import torch
import torch.nn as nn

import arm_config as cfg


class ResidualBlock(nn.Module):
    """
    One pre-activation-style residual block:

        x -> Linear -> GELU -> Linear -> (+x) -> GELU

    Why residual connections at all, for a plain coordinate-regression
    MLP? Because IK for a redundant-feeling 5-DOF chain is a highly
    non-linear, locally many-to-one mapping (nearby Cartesian targets can
    require quite different joint combinations near singularities of the
    arm). Skip connections give the optimizer a direct gradient path back
    to early layers, which in practice makes deeper nets like this one
    (6 blocks x 512 units) actually trainable instead of vanishing-
    gradient soup, and lets each block learn a *residual correction* to
    an already-reasonable estimate rather than the whole mapping from
    scratch.

    GELU (vs ReLU) is used because the target manifold is smooth
    (joint angles vary continuously with position) -- GELU's smooth,
    non-zero gradient everywhere avoids the "dead unit" issue ReLU can
    have and empirically gives smoother learned functions for this kind
    of continuous regression, which matters when we later want a smooth
    joint trajectory out of stage 3.
    """

    def __init__(self, dim: int):
        super().__init__()
        self.fc1 = nn.Linear(dim, dim)
        self.fc2 = nn.Linear(dim, dim)
        self.act = nn.GELU()

    def forward(self, x):
        residual = x
        h = self.act(self.fc1(x))
        h = self.fc2(h)
        return self.act(h + residual)


class ResidualMLPIK(nn.Module):
    """
    Cartesian (x, y, z, vx, vy) -> joint angles (q1..q5), normalized to
    [-1, 1] via a Tanh output head (see arm_config.JOINT_LIMITS for the
    physical denormalization).

    Architecture: input projection -> N residual blocks -> output head.
    """

    def __init__(self,
                 input_dim: int = cfg.INPUT_DIM,
                 hidden_dim: int = cfg.HIDDEN_DIM,
                 output_dim: int = cfg.OUTPUT_DIM,
                 num_blocks: int = cfg.NUM_RESIDUAL_BLOCKS):
        super().__init__()
        self.input_proj = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.GELU(),
        )
        self.blocks = nn.ModuleList([ResidualBlock(hidden_dim) for _ in range(num_blocks)])
        self.output_head = nn.Sequential(
            nn.Linear(hidden_dim, output_dim),
            nn.Tanh(),   # bounds every output to exactly [-1, 1]
        )

    def forward(self, x):
        h = self.input_proj(x)
        for block in self.blocks:
            h = block(h)
        return self.output_head(h)


def denormalize_joints(pred_norm: torch.Tensor, joint_lo: torch.Tensor, joint_hi: torch.Tensor) -> torch.Tensor:
    """Map network output in [-1, 1] back to physical radians using URDF limits."""
    return (pred_norm + 1.0) * 0.5 * (joint_hi - joint_lo) + joint_lo


def normalize_joints(joint_rad: torch.Tensor, joint_lo: torch.Tensor, joint_hi: torch.Tensor) -> torch.Tensor:
    """Inverse of denormalize_joints: physical radians -> [-1, 1]."""
    return 2.0 * (joint_rad - joint_lo) / (joint_hi - joint_lo) - 1.0
