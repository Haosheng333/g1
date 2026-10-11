"""2D grid helpers for grid_composer (no rospy import: unit-testable)."""
import os

import numpy as np
import yaml

# int8 values written into the OccupancyGrid /map  (doc 4.3: WALKABLE / RISK /
# DANGER / OBSTACLE).  Kept inside 0..100 so rviz's Map display still renders it.
UNKNOWN = -1
WALKABLE = 0
RISK = 33
DANGER = 66
OBSTACLE = 100


def read_pgm(path):
    """Binary (P5) PGM -> uint8 array (H, W), row 0 = top of the image."""
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1  # single whitespace after maxval
    if tokens[0] != b"P5":
        raise ValueError("%s is not a binary PGM (P5)" % path)
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError("16-bit PGM not supported")
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w)


def load_map_yaml(yaml_path):
    """map_server yaml + pgm -> (grid, resolution, origin_x, origin_y).

    grid is int8 with row 0 = LOWEST y (OccupancyGrid order), values
    UNKNOWN / WALKABLE / OBSTACLE.
    """
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta["image"]))
    negate = int(meta.get("negate", 0))
    occ_t = float(meta.get("occupied_thresh", 0.65))
    free_t = float(meta.get("free_thresh", 0.196))
    p = img.astype(np.float64) / 255.0
    occ = p if negate else 1.0 - p
    grid = np.full(img.shape, UNKNOWN, dtype=np.int8)
    grid[occ > occ_t] = OBSTACLE
    grid[occ < free_t] = WALKABLE
    grid = grid[::-1, :].copy()  # image row 0 is the top (max y); OccupancyGrid row 0 is min y
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return grid, float(meta["resolution"]), float(origin[0]), float(origin[1])


def fill_unknown_holes(grid, iterations=3):
    """Turn small UNKNOWN holes inside free space into WALKABLE.

    A LiDAR floor is never covered cell-by-cell, so a saved map has speckled unknown
    cells inside rooms; the planner treats unknown as blocked.  Morphological CLOSING of
    the free set fills holes up to ~2*iterations cells wide but does not grow free space
    into real frontiers (unexplored areas stay unknown) and never overwrites obstacles.
    """
    from scipy.ndimage import binary_closing

    if iterations <= 0:
        return grid
    free = grid == WALKABLE
    closed = binary_closing(np.pad(free, iterations + 1), structure=np.ones((3, 3), bool),
                            iterations=iterations)[iterations + 1:-(iterations + 1),
                                                   iterations + 1:-(iterations + 1)]
    out = grid.copy()
    out[closed & (grid == UNKNOWN)] = WALKABLE
    return out


def compose_tiers(grid, resolution, danger_m=0.25, risk_m=0.50, hole_fill_iterations=3):
    """Binary grid (-1/0/100) -> tiered grid (-1/0/33/66/100) by distance to obstacles."""
    from scipy.ndimage import distance_transform_edt

    grid = fill_unknown_holes(grid, hole_fill_iterations)
    occ = grid == OBSTACLE
    out = np.full(grid.shape, UNKNOWN, dtype=np.int8)
    out[grid == WALKABLE] = WALKABLE
    if occ.any():
        dist = distance_transform_edt(~occ) * resolution
        free = grid == WALKABLE
        out[free & (dist <= risk_m)] = RISK
        out[free & (dist <= danger_m)] = DANGER
    out[occ] = OBSTACLE
    return out
