"""Semantic goals: semantic.yaml loader + spiral search for a standable approach pose.

Pure python (no rospy) so it can be unit-tested.  `costmap` is anything with
is_free(mx,my), world_to_map(wx,wy), map_to_world(mx,my) and .resolution
(see costmap.OccupancyCostmap); `planner` has plan(costmap, start_xy, goal_xy).
"""
import math

import yaml


def load_semantic(path):
    """semantic.yaml -> {lower-case name: {x, y, yaw, approach_yaw|None}}."""
    if not path:
        raise ValueError("semantic_file parameter is empty")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    out = {}
    for name, v in (data.get("waypoints") or {}).items():
        approach_yaw = v.get("approach_yaw")
        out[str(name).strip().lower()] = {
            "x": float(v["x"]),
            "y": float(v["y"]),
            "yaw": float(v.get("yaw", 0.0)),
            "approach_yaw": None if approach_yaw is None else float(approach_yaw),
        }
    return out


def lookup_object(semantic, class_name):
    return semantic.get(str(class_name).strip().lower())


def _ring(cx, cy, r):
    """Cells at Chebyshev distance exactly r from (cx, cy)."""
    if r == 0:
        yield (cx, cy)
        return
    for dx in range(-r, r + 1):
        yield (cx + dx, cy - r)
        yield (cx + dx, cy + r)
    for dy in range(-r + 1, r):
        yield (cx - r, cy + dy)
        yield (cx + r, cy + dy)


def _standable(costmap, mx, my, radius_cells):
    if hasattr(costmap, "standable"):          # tiered Costmap: distance-transform based, any heading
        return costmap.standable(mx, my)
    r2 = radius_cells * radius_cells
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= r2 and not costmap.is_free(mx + dx, my + dy):
                return False
    return True


def find_approach_pose(costmap, planner, obj_xy, approach_distance, from_xy,
                       approach_yaw=None, clearance=0.15, max_search=1.5,
                       max_attempts=25):
    """Spiral-search the costmap for a standable, reachable pose near the object.

    The ideal pose is `approach_distance` from the object, on the side the robot
    comes from (or in direction `approach_yaw` if given).  The search expands in
    square rings around it; the first standable cell the planner can reach wins.
    Returns (x, y, yaw) with yaw facing the object, or None.
    """
    ox, oy = obj_xy
    if approach_yaw is not None:
        dx, dy = math.cos(approach_yaw), math.sin(approach_yaw)
    else:
        dx, dy = from_xy[0] - ox, from_xy[1] - oy
        n = math.hypot(dx, dy)
        dx, dy = (1.0, 0.0) if n < 1e-6 else (dx / n, dy / n)
    ix, iy = ox + approach_distance * dx, oy + approach_distance * dy
    cx, cy = costmap.world_to_map(ix, iy)
    rad = max(0, int(math.ceil(clearance / costmap.resolution)))
    max_r = int(math.ceil(max_search / costmap.resolution))
    attempts = 0
    for r in range(max_r + 1):
        cells = [c for c in _ring(cx, cy, r) if _standable(costmap, c[0], c[1], rad)]
        cells.sort(key=lambda c: (c[0] - cx) ** 2 + (c[1] - cy) ** 2)
        for mx, my in cells:
            wx, wy = costmap.map_to_world(mx, my)
            if planner is not None:
                if attempts >= max_attempts:
                    return None
                attempts += 1
                if not planner.plan(costmap, from_xy, (wx, wy)):
                    continue
            return (wx, wy, math.atan2(oy - wy, ox - wx))
    return None
