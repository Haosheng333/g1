"""ROS-free helpers for handover_node.py (mirrors main_process's own)."""

from __future__ import annotations

import math


def unflatten(flat, count):
    """N x 5 joint waypoints from a flat list; [] if the shape is wrong."""
    flat = list(flat)
    if count <= 0 or len(flat) != 5 * count:
        return []
    return [flat[i * 5:(i + 1) * 5] for i in range(count)]


def interp_joint_waypoints(q_from, q_to, max_step_deg):
    """Joint-space interpolation ending at q_to, no joint moving more than
    max_step_deg per waypoint; q_from itself is not included."""
    q_from, q_to = list(q_from), list(q_to)
    if len(q_from) != len(q_to) or max_step_deg is None or max_step_deg <= 0.0:
        return [q_to]
    max_delta = max(abs(float(b) - float(a)) for a, b in zip(q_from, q_to))
    if max_delta <= 1e-6:
        return [q_to]
    n = max(1, int(math.ceil(max_delta / math.radians(float(max_step_deg)))))
    return [[a + (b - a) * k / n for a, b in zip(q_from, q_to)] for k in range(1, n + 1)]
