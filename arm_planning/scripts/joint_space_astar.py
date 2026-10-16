#!/usr/bin/env python3
"""Four-dimensional joint-space A* for the Unitree G1 arm.

Search dimensions: shoulder pitch, shoulder roll, shoulder yaw, elbow.
Wrist roll is appended as a smooth final rotation after the 4D path.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Tuple
import argparse
import heapq
import itertools
import json
import math

import numpy as np
import yaml

# typing.Tuple, not tuple[...]: this is a RUNTIME assignment, not an
# annotation, so `from __future__ import annotations` does not defer it and
# PEP 585 subscripting of the builtin raises on Python 3.8 -- which is the
# interpreter ROS Noetic runs. Before this, importing this module (and so
# edge_checked_astar and collision_aware_planner, which import it) failed
# outright on the deployment interpreter.
State = Tuple[int, int, int, int]
BlockedFn = Callable[[State], bool]


@dataclass
class PlanResult:
    success: bool
    message: str
    path: np.ndarray | None
    coarse_states: list[State]
    nodes_expanded: int
    used_fallback: bool


class JointGrid:
    def __init__(self, lower: Iterable[float], upper: Iterable[float], resolution_deg: float = 4.0):
        self.lower = np.asarray(list(lower), dtype=float)[:4]
        self.upper = np.asarray(list(upper), dtype=float)[:4]
        self.resolution = math.radians(resolution_deg)
        if np.any(self.upper <= self.lower):
            raise ValueError("Every upper joint limit must exceed its lower limit")
        self.counts = np.floor((self.upper - self.lower) / self.resolution).astype(int) + 1

    def joints_to_state(self, joints: Iterable[float]) -> State:
        q = np.asarray(list(joints), dtype=float)[:4]
        cells = np.rint((q - self.lower) / self.resolution).astype(int)
        cells = np.clip(cells, 0, self.counts - 1)
        return tuple(int(v) for v in cells)

    def state_to_joints(self, state: State) -> np.ndarray:
        return self.lower + np.asarray(state, dtype=float) * self.resolution

    def valid(self, state: State) -> bool:
        s = np.asarray(state)
        return bool(np.all(s >= 0) and np.all(s < self.counts))

    def neighbors(self, state: State, movable_axes=(0, 1, 2, 3)):
        for axis in movable_axes:
            for step in (-1, 1):
                candidate = list(state)
                candidate[axis] += step
                candidate = tuple(candidate)
                if self.valid(candidate):
                    yield candidate, axis


def weighted_manhattan(state: State, goal: State, weights=(1.0, 1.0, 1.0, 1.0)) -> float:
    return float(sum(w * abs(a - b) for a, b, w in zip(state, goal, weights)))


def reconstruct(came_from: dict[State, State], current: State) -> list[State]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar_search(
    grid: JointGrid,
    start: State,
    goal: State,
    is_blocked: BlockedFn | None = None,
    movable_axes=(0, 1, 2, 3),
    move_weights=(1.0, 1.0, 1.0, 1.0),
    max_expansions=1_000_000,
):
    blocked = is_blocked or (lambda _: False)
    if not grid.valid(start) or not grid.valid(goal):
        return None, 0, "start or goal lies outside the grid"
    if blocked(start):
        return None, 0, "start state is blocked"
    if blocked(goal):
        return None, 0, "goal state is blocked"
    if start == goal:
        return [start], 0, "success"

    counter = itertools.count()
    open_heap = []
    g_score = {start: 0.0}
    came_from: dict[State, State] = {}
    h0 = weighted_manhattan(start, goal, move_weights)
    heapq.heappush(open_heap, (h0, h0, next(counter), start))
    closed: set[State] = set()
    expanded = 0

    while open_heap:
        _, _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return reconstruct(came_from, current), expanded, "success"
        closed.add(current)
        expanded += 1
        if expanded > max_expansions:
            return None, expanded, "maximum A* expansions exceeded"

        for neighbor, changed_axis in grid.neighbors(current, movable_axes):
            if neighbor in closed or blocked(neighbor):
                continue
            tentative = g_score[current] + float(move_weights[changed_axis])
            if tentative < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                h = weighted_manhattan(neighbor, goal, move_weights)
                # Lower f first, then lower h, then insertion order.
                heapq.heappush(open_heap, (tentative + h, h, next(counter), neighbor))
    return None, expanded, "no path found"


def remove_near_duplicate_waypoints(path: np.ndarray, threshold_rad: float = 0.03) -> np.ndarray:
    if len(path) <= 2:
        return path.copy()
    kept = [path[0]]
    for point in path[1:-1]:
        if np.max(np.abs(point - kept[-1])) >= threshold_rad:
            kept.append(point)
    kept.append(path[-1])
    return np.asarray(kept)


def interpolate_joint_path(path: np.ndarray, max_step_rad: float = math.radians(1.0)) -> np.ndarray:
    if len(path) == 0:
        return path.copy()
    output = [path[0].copy()]
    for start, end in zip(path[:-1], path[1:]):
        steps = max(1, int(math.ceil(np.max(np.abs(end - start)) / max_step_rad)))
        for i in range(1, steps + 1):
            output.append(start + (end - start) * (i / steps))
    return np.asarray(output)


def states_to_five_joint_path(
    grid: JointGrid,
    states: list[State],
    q_start: np.ndarray,
    q_goal: np.ndarray,
    merge_threshold_rad: float = 0.03,
    max_step_rad: float = math.radians(1.0),
) -> np.ndarray:
    four = np.asarray([grid.state_to_joints(s) for s in states])
    # Preserve the exact continuous start and goal. A one-state grid path still
    # needs two continuous endpoints when q_start and q_goal differ inside a cell.
    if len(four) == 1:
        four = np.vstack((q_start[:4], q_goal[:4]))
    else:
        four[0] = q_start[:4]
        four[-1] = q_goal[:4]
    wrist = np.full((len(four), 1), q_start[4], dtype=float)
    coarse = np.hstack((four, wrist))
    # Finish wrist roll at the verified target. Simplify before interpolation so
    # the final path still respects the maximum one-degree step requirement.
    if not np.isclose(q_start[4], q_goal[4]):
        final = coarse[-1].copy()
        final[4] = q_goal[4]
        coarse = np.vstack((coarse, final))
    coarse = remove_near_duplicate_waypoints(coarse, merge_threshold_rad)
    dense = interpolate_joint_path(coarse, max_step_rad)
    dense[-1] = q_goal
    return dense


def staged_plan(grid, start, goal, blocked, max_expansions):
    # Stage 1: hold ShoulderYaw at its current cell while moving pitch, roll, elbow.
    stage1_goal = (goal[0], goal[1], start[2], goal[3])
    p1, n1, m1 = astar_search(grid, start, stage1_goal, blocked, (0, 1, 3), (1, 1, 1, 1), max_expansions)
    if p1 is None:
        return None, n1, f"stage 1 failed: {m1}"
    # Stage 2: adjust only ShoulderYaw.
    p2, n2, m2 = astar_search(grid, stage1_goal, goal, blocked, (2,), (1, 1, 1, 1), max_expansions)
    if p2 is None:
        return None, n1 + n2, f"stage 2 failed: {m2}"
    return p1 + p2[1:], n1 + n2, "success"


def plan_joint_path(
    q_start,
    q_goal,
    lower_limits,
    upper_limits,
    is_blocked: BlockedFn | None = None,
    resolution_deg=4.0,
    max_expansions=1_000_000,
) -> PlanResult:
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    lower = np.asarray(lower_limits, dtype=float)
    upper = np.asarray(upper_limits, dtype=float)
    if q_start.shape != (5,) or q_goal.shape != (5,):
        return PlanResult(False, "start and goal must each contain five joints", None, [], 0, False)
    if not np.all(np.isfinite(q_start)) or not np.all(np.isfinite(q_goal)):
        return PlanResult(False, "start and goal must be finite", None, [], 0, False)
    if np.any(q_start < lower) or np.any(q_start > upper) or np.any(q_goal < lower) or np.any(q_goal > upper):
        return PlanResult(False, "start or goal violates joint limits", None, [], 0, False)

    grid = JointGrid(lower, upper, resolution_deg)
    start = grid.joints_to_state(q_start)
    goal = grid.joints_to_state(q_goal)
    blocked = is_blocked or (lambda _: False)

    states, expanded, message = staged_plan(grid, start, goal, blocked, max_expansions)
    fallback = False
    if states is None:
        fallback = True
        states, extra, fallback_message = astar_search(
            grid, start, goal, blocked,
            movable_axes=(0, 1, 2, 3),
            move_weights=(1.0, 1.0, 3.0, 1.0),
            max_expansions=max_expansions,
        )
        expanded += extra
        if states is None:
            return PlanResult(False, f"staged plan failed ({message}); fallback failed ({fallback_message})", None, [], expanded, True)

    # Goal guard: final grid state must be no more than one cell away on every axis.
    if np.any(np.abs(np.asarray(states[-1]) - np.asarray(goal)) > 1):
        return PlanResult(False, "goal guard failed: final state is more than one grid cell from goal", None, states, expanded, fallback)

    dense = states_to_five_joint_path(grid, states, q_start, q_goal)
    if np.any(dense < lower - 1e-9) or np.any(dense > upper + 1e-9):
        return PlanResult(False, "post-processed path violates joint limits", None, states, expanded, fallback)
    if len(dense) > 1 and np.max(np.abs(np.diff(dense, axis=0))) > math.radians(1.0) + 1e-9:
        return PlanResult(False, "post-processed path exceeds one-degree step limit", None, states, expanded, fallback)
    return PlanResult(True, "success", dense, states, expanded, fallback)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--arm", choices=("left", "right"), required=True)
    p.add_argument("--start", nargs=5, type=float, required=True)
    p.add_argument("--goal", nargs=5, type=float, required=True)
    p.add_argument("--config", type=Path, default=Path("config/arm_kinematics.yaml"))
    p.add_argument("--output", type=Path, default=Path("planned_joint_path.npy"))
    args = p.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    result = plan_joint_path(args.start, args.goal, config[args.arm]["lower_limits"], config[args.arm]["upper_limits"])
    if result.success:
        np.save(args.output, result.path)
    print(json.dumps({
        "success": result.success,
        "message": result.message,
        "waypoints": 0 if result.path is None else len(result.path),
        "coarse_states": len(result.coarse_states),
        "nodes_expanded": result.nodes_expanded,
        "used_fallback": result.used_fallback,
        "output": str(args.output) if result.success else None,
    }, indent=2))
    raise SystemExit(0 if result.success else 2)


if __name__ == "__main__":
    main()
