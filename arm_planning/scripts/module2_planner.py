#!/usr/bin/env python3
"""Single orchestrator-facing Module 2 point-to-point planning interface."""
from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any
import argparse
import json
import threading

import numpy as np
import yaml

from guarded_ik_solver import GuardedIKSolver
from collision_aware_planner import plan_collision_aware
from collision_aware_ik import solve_collision_aware_ik
from g1_collision_probe import G1CollisionProbe


@dataclass
class PlanToPoseResult:
    success: bool
    message: str
    arm: str
    joint_names: list[str]
    joint_path: list[list[float]]
    target_pose: list[float]
    verified_goal_joints: list[float]
    position_error_m: float | None
    rx_error_rad: float | None
    rz_error_rad: float | None
    selected_grid_state: list[int] | None
    exact_goal_cell_blocked: bool | None
    nodes_expanded: int
    used_fallback: bool


class Module2Planner:
    """Loads both arms once and exposes a thread-safe plan_to_pose method."""

    def __init__(
        self,
        config_path: str | Path = "config/arm_kinematics.yaml",
        left_checkpoint: str | Path = "models_hybrid_1m/g1_left_hybrid_best.pt",
        right_checkpoint: str | Path = "models_hybrid_1m/g1_right_hybrid_best.pt",
        left_mask: str | Path = "collision_masks/left_static_collision_mask.npy",
        right_mask: str | Path = "collision_masks/right_static_collision_mask.npy",
        project_root: str | Path = ".",
        max_position_error_m: float = 0.01,
        max_rx_error_rad: float = 0.10,
        max_rz_error_rad: float = 0.15,
        search_radius: int = 3,
    ):
        self.config_path = Path(config_path)
        self.project_root = Path(project_root)
        self.search_radius = int(search_radius)
        if self.search_radius < 0:
            raise ValueError("search_radius must be nonnegative")

        self.config = yaml.safe_load(self.config_path.read_text(encoding="utf-8"))
        self.mask_paths = {
            "left": Path(left_mask),
            "right": Path(right_mask),
        }
        checkpoint_paths = {
            "left": Path(left_checkpoint),
            "right": Path(right_checkpoint),
        }

        required = [self.config_path, *self.mask_paths.values(), *checkpoint_paths.values()]
        missing = [str(path) for path in required if not path.exists()]
        if missing:
            raise FileNotFoundError("Missing Module 2 assets: " + ", ".join(missing))

        self.solvers = {
            arm: GuardedIKSolver(
                checkpoint_path=checkpoint_paths[arm],
                arm=arm,
                config_path=self.config_path,
                max_position_error_m=max_position_error_m,
                max_rx_error_rad=max_rx_error_rad,
                max_rz_error_rad=max_rz_error_rad,
                device="cpu",
            )
            for arm in ("left", "right")
        }
        # FCL collision objects are mutable, so serialize planner calls per arm.
        self._locks = {"left": threading.Lock(), "right": threading.Lock()}

    def _failure(self, arm: str, pose: Any, message: str) -> PlanToPoseResult:
        target = []
        try:
            candidate = np.asarray(pose, dtype=float)
            if candidate.shape == (5,) and np.all(np.isfinite(candidate)):
                target = candidate.tolist()
        except (TypeError, ValueError):
            pass
        joint_names = self.config.get(arm, {}).get("joint_names", []) if arm in ("left", "right") else []
        return PlanToPoseResult(False, message, arm, list(joint_names), [], target, [], None, None, None, None, None, 0, False)

    def plan_to_pose(self, arm: str, current_joints, target_pose) -> PlanToPoseResult:
        """Plan from current 5-joint state to [x,y,z,rx,rz]. Never returns an unsafe partial path."""
        if arm not in ("left", "right"):
            return self._failure(str(arm), target_pose, "arm must be 'left' or 'right'")

        try:
            current = np.asarray(current_joints, dtype=float)
            target = np.asarray(target_pose, dtype=float)
        except (TypeError, ValueError):
            return self._failure(arm, target_pose, "current_joints and target_pose must be numeric")

        if current.shape != (5,) or not np.all(np.isfinite(current)):
            return self._failure(arm, target_pose, "current_joints must contain five finite values")
        if target.shape != (5,) or not np.all(np.isfinite(target)):
            return self._failure(arm, target_pose, "target_pose must contain five finite values [x,y,z,rx,rz]")

        arm_cfg = self.config[arm]
        lower = np.asarray(arm_cfg["lower_limits"], dtype=float)
        upper = np.asarray(arm_cfg["upper_limits"], dtype=float)
        if np.any(current < lower) or np.any(current > upper):
            return self._failure(arm, target, "current_joints violate configured joint limits")

        with self._locks[arm]:
            ik = self.solvers[arm].solve(target)
            if not ik.success:
                return self._failure(arm, target, "IK verification failed: " + ik.message)

            goal = np.asarray(ik.joints_rad, dtype=float)
            planned = plan_collision_aware(
                q_start=current,
                q_goal=goal,
                arm=arm,
                config=self.config,
                mask_path=self.mask_paths[arm],
                project_root=self.project_root,
                resolution_deg=4.0,
                search_radius=self.search_radius,
            )
            if (not planned.success) and planned.message == 'exact IK goal is colliding':
                probe=G1CollisionProbe(self.project_root,arm,self.config)
                alternate=solve_collision_aware_ik(
                    target,arm,self.config,probe,initial_joints=goal,
                    max_position_error_m=self.solvers[arm].max_position_error_m,
                    max_rx_error_rad=self.solvers[arm].max_rx_error_rad,
                    max_rz_error_rad=self.solvers[arm].max_rz_error_rad,
                )
                if alternate.success:
                    goal=np.asarray(alternate.joints,dtype=float)
                    ik.position_error_m=alternate.position_error_m
                    ik.rx_error_rad=alternate.rx_error_rad
                    ik.rz_error_rad=alternate.rz_error_rad
                    planned=plan_collision_aware(
                        q_start=current,q_goal=goal,arm=arm,config=self.config,
                        mask_path=self.mask_paths[arm],project_root=self.project_root,
                        resolution_deg=4.0,search_radius=self.search_radius,
                    )
            if not planned.success or planned.path is None:
                return self._failure(arm, target, "Collision-aware planning failed: " + planned.message)

            path = np.asarray(planned.path, dtype=float)
            if path.ndim != 2 or path.shape[1] != 5 or len(path) == 0:
                return self._failure(arm, target, "Planner returned an invalid path shape")
            if not np.all(np.isfinite(path)):
                return self._failure(arm, target, "Planner returned non-finite joint values")
            if not np.allclose(path[0], current, atol=1e-9):
                return self._failure(arm, target, "Planner path does not start at current_joints")
            if not np.allclose(path[-1], goal, atol=1e-9):
                return self._failure(arm, target, "Planner path does not end at the verified IK goal")

            return PlanToPoseResult(
                success=True,
                message="success",
                arm=arm,
                joint_names=list(arm_cfg["joint_names"]),
                joint_path=path.tolist(),
                target_pose=target.tolist(),
                verified_goal_joints=goal.tolist(),
                position_error_m=float(ik.position_error_m),
                rx_error_rad=float(ik.rx_error_rad),
                rz_error_rad=float(ik.rz_error_rad),
                selected_grid_state=list(planned.selected_grid_state) if planned.selected_grid_state is not None else None,
                exact_goal_cell_blocked=bool(planned.exact_goal_cell_blocked),
                nodes_expanded=int(planned.nodes_expanded),
                used_fallback=bool(planned.used_fallback),
            )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--arm", choices=["left", "right"], required=True)
    p.add_argument("--current-joints", nargs=5, type=float, required=True)
    p.add_argument("--target-pose", nargs=5, type=float, required=True)
    p.add_argument("--config", type=Path, default=Path("config/arm_kinematics.yaml"))
    p.add_argument("--left-checkpoint", type=Path, default=Path("models_hybrid_1m/g1_left_hybrid_best.pt"))
    p.add_argument("--right-checkpoint", type=Path, default=Path("models_hybrid_1m/g1_right_hybrid_best.pt"))
    p.add_argument("--left-mask", type=Path, default=Path("collision_masks/left_static_collision_mask.npy"))
    p.add_argument("--right-mask", type=Path, default=Path("collision_masks/right_static_collision_mask.npy"))
    p.add_argument("--project-root", type=Path, default=Path("."))
    p.add_argument("--output-json", type=Path)
    args = p.parse_args()

    planner = Module2Planner(
        config_path=args.config,
        left_checkpoint=args.left_checkpoint,
        right_checkpoint=args.right_checkpoint,
        left_mask=args.left_mask,
        right_mask=args.right_mask,
        project_root=args.project_root,
    )
    result = planner.plan_to_pose(args.arm, args.current_joints, args.target_pose)
    text = json.dumps(asdict(result), indent=2)
    print(text)
    if args.output_json:
        args.output_json.write_text(text, encoding="utf-8")
    raise SystemExit(0 if result.success else 2)


if __name__ == "__main__":
    main()
