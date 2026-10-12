"""Layer (1) of doc section 4.3: the COSTMAP.

    /map (tiered, ~1 Hz)  +  /obstacle_cloud (live lethal layer, ~10 Hz)
        -> per-layer distance transforms
        -> tier penalties + soft decay                    (used by the global planner)
        -> robot footprint = 5 circles of r = 0.10 m      (used by the local planner)

Pure numpy/scipy, no rospy: unit-testable offline.

Design
------
* `Costmap` is an IMMUTABLE snapshot.  ROS callbacks build a new one and swap the
  reference, so the planner thread never sees a half-updated grid.
* `CostmapManager` owns the current snapshot, the static layer and a short history of
  live points (a single Mid-360 scan is sparse, so ~0.6 s of scans are unioned).

Grid conventions (match g1_slam/grid_composer): int8, row 0 = lowest y, values
    -1 UNKNOWN   0 WALKABLE   33 RISK   66 DANGER   100 OBSTACLE
"""
import collections
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

UNKNOWN, WALKABLE, RISK, DANGER, OBSTACLE = -1, 0, 33, 66, 100

DEFAULTS = {
    # robot footprint: `footprint_circles` circles of `circle_radius`, centred on the body
    # axis line y in [-footprint_half_width, +footprint_half_width] (doc: 5 circles, r = 0.10 m)
    "footprint_circles": 5,
    "footprint_half_width": 0.20,
    "circle_radius": 0.10,
    "collision_margin": 0.02,        # footprint clearance (m) below which a pose collides
    # global planner
    "astar_clearance": 0.35,         # footprint lateral reach (0.20 + 0.10 m) + 0.05 m margin: cells closer to a lethal cell are not traversable
    "standable_clearance": 0.40,     # needed around an approach pose (any heading)
    "unknown_is_lethal": True,
    # tier penalties (extra cost per metre = 0.1 * penalty) and soft decay of live/static obstacles
    "risk_penalty": 6.0,
    "danger_penalty": 30.0,
    "decay_gain": 20.0,
    "decay_length": 0.30,
    # live layer
    "live_ignore_near_static": 0.10,  # live points this close to a static obstacle ARE the static map
    "live_hold": 0.6,                 # seconds of live scans that are unioned
}


def make_config(overrides=None):
    cfg = dict(DEFAULTS)
    if overrides:
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            raise KeyError("unknown costmap parameter(s): %s" % sorted(unknown))
        cfg.update(overrides)
    return cfg


class Costmap:
    """Immutable snapshot.  Build with CostmapManager (or `Costmap.from_grid` in tests)."""

    def __init__(self, grid, resolution, origin_x, origin_y, live_mask, static_dist, cfg):
        self.cfg = cfg
        self.grid = grid
        self.resolution = float(resolution)
        self.origin_x, self.origin_y = float(origin_x), float(origin_y)
        self.height, self.width = grid.shape
        self.live_mask = live_mask

        static_lethal = grid == OBSTACLE
        if cfg["unknown_is_lethal"]:
            static_lethal = static_lethal | (grid == UNKNOWN)
        if live_mask is not None and live_mask.any():
            dist = distance_transform_edt(~(static_lethal | live_mask)) * self.resolution
        else:
            dist = static_dist
        self.dist = dist.astype(np.float32)                    # metres to nearest lethal cell

        tier_pen = np.zeros(grid.shape, np.float32)
        tier_pen[grid == RISK] = cfg["risk_penalty"]
        tier_pen[grid >= DANGER] = cfg["danger_penalty"]
        decay = cfg["decay_gain"] * np.exp(-self.dist / cfg["decay_length"])
        self.penalty = (tier_pen + decay).astype(np.float32)   # soft-decay cost layer
        self.blocked = (self.dist < cfg["astar_clearance"]) | (grid >= DANGER) | static_lethal
        if live_mask is not None:
            self.blocked |= live_mask

        n = int(cfg["footprint_circles"])
        hw = float(cfg["footprint_half_width"])
        self.circle_y = np.linspace(-hw, hw, n) if n > 1 else np.zeros(1)
        self._res_inv = 1.0 / self.resolution

    # ------------------------------------------------------------------ build
    @staticmethod
    def static_distance(grid, resolution, cfg):
        lethal = grid == OBSTACLE
        if cfg["unknown_is_lethal"]:
            lethal = lethal | (grid == UNKNOWN)
        return distance_transform_edt(~lethal) * resolution

    @classmethod
    def from_grid(cls, grid, resolution, origin_x=0.0, origin_y=0.0, **overrides):
        cfg = make_config(overrides)
        grid = np.asarray(grid, np.int8)
        return cls(grid, resolution, origin_x, origin_y, None, cls.static_distance(grid, resolution, cfg), cfg)

    # -------------------------------------------------------------- coordinates
    def world_to_map(self, wx, wy):
        return (int(math.floor((wx - self.origin_x) * self._res_inv)),
                int(math.floor((wy - self.origin_y) * self._res_inv)))

    def map_to_world(self, mx, my):
        return (self.origin_x + (mx + 0.5) * self.resolution,
                self.origin_y + (my + 0.5) * self.resolution)

    def in_bounds(self, mx, my):
        return 0 <= mx < self.width and 0 <= my < self.height

    # ------------------------------------------------------------------ queries
    def is_free(self, mx, my):
        """Traversable for the GLOBAL planner (point robot with clearance)."""
        return self.in_bounds(mx, my) and not self.blocked[my, mx]

    def standable(self, mx, my):
        """Room to stand at any heading (used by the semantic approach-pose search)."""
        return (self.in_bounds(mx, my) and not self.blocked[my, mx]
                and self.dist[my, mx] >= self.cfg["standable_clearance"])

    def penalty_at(self, mx, my):
        return float(self.penalty[my, mx]) if self.in_bounds(mx, my) else 1e6

    def dist_many(self, xs, ys):
        """Distance (m) to the nearest lethal cell CENTRE for arrays of world points, bilinearly
        interpolated between cell centres (a plain lookup is quantised to the 5 cm grid, which let the
        robot creep ~5 cm into a wall without the check noticing).  Outside the map = 0."""
        xs, ys = np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64)
        fx = (xs - self.origin_x) * self._res_inv - 0.5          # continuous cell-centre coordinates
        fy = (ys - self.origin_y) * self._res_inv - 0.5
        x0, y0 = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
        tx, ty = (fx - x0).astype(np.float32), (fy - y0).astype(np.float32)
        inside = (x0 >= 0) & (x0 + 1 < self.width) & (y0 >= 0) & (y0 + 1 < self.height)
        xi, yi = np.clip(x0, 0, self.width - 2), np.clip(y0, 0, self.height - 2)
        d = self.dist
        v = ((1 - tx) * (1 - ty) * d[yi, xi] + tx * (1 - ty) * d[yi, xi + 1]
             + (1 - tx) * ty * d[yi + 1, xi] + tx * ty * d[yi + 1, xi + 1])
        # points in the outermost half cell: fall back to the nearest cell value
        edge = ~inside & (xs >= self.origin_x) & (ys >= self.origin_y) \
            & (xs < self.origin_x + self.width * self.resolution) & (ys < self.origin_y + self.height * self.resolution)
        if edge.any():
            ex = np.clip(np.floor((xs - self.origin_x) * self._res_inv).astype(np.int64), 0, self.width - 1)
            ey = np.clip(np.floor((ys - self.origin_y) * self._res_inv).astype(np.int64), 0, self.height - 1)
            v = np.where(edge, d[ey, ex], v)
            inside = inside | edge
        return np.where(inside, v, 0.0).astype(np.float32)

    def dist_at(self, wx, wy):
        return float(self.dist_many(np.array([wx]), np.array([wy]))[0])

    # ---------------------------------------------------------------- footprint
    def footprint_clearance_many(self, x, y, yaw):
        """Min over the footprint circles of (distance to nearest lethal cell - circle radius).
        x, y, yaw: equally shaped arrays (any shape).  Negative or < collision_margin => collision."""
        x, y, yaw = np.asarray(x), np.asarray(y), np.asarray(yaw)
        s, c = np.sin(yaw), np.cos(yaw)
        clear = None
        for yi in self.circle_y:
            d = self.dist_many(x - yi * s, y + yi * c)
            clear = d if clear is None else np.minimum(clear, d)
        # `dist` is measured to the lethal cell's CENTRE; the obstacle surface is half a cell closer.
        return clear - 0.5 * self.resolution - self.cfg["circle_radius"]

    def footprint_clearance(self, x, y, yaw):
        return float(self.footprint_clearance_many(np.array([x]), np.array([y]), np.array([yaw]))[0])

    def footprint_collision(self, x, y, yaw):
        return self.footprint_clearance(x, y, yaw) < self.cfg["collision_margin"]

    # ------------------------------------------------------------------- debug
    def to_occupancy_data(self):
        """int8 0..100 cost picture for rviz (/navigation/costmap)."""
        out = np.minimum(self.penalty * 2.0, 99.0)
        out[self.blocked] = 100
        out[self.grid == UNKNOWN] = -1
        return out.astype(np.int8)


class CostmapManager:
    """Owns the static layer + live history and publishes immutable `Costmap` snapshots."""

    def __init__(self, **overrides):
        self.cfg = make_config(overrides)
        self.state = None
        self._static = None          # (grid, res, ox, oy, static_dist)
        self._hist = collections.deque()   # (time, xy array)
        self._last_live_key = None

    def set_static(self, grid, resolution, origin_x, origin_y):
        grid = np.asarray(grid, np.int8)
        sd = Costmap.static_distance(grid, resolution, self.cfg)
        self._static = (grid, resolution, origin_x, origin_y, sd)
        self._rebuild()

    def set_live_points(self, xy, now):
        """xy: (N, 2) world points of the CURRENT scan, `now`: seconds."""
        if self._static is None:
            return
        self._hist.append((now, np.asarray(xy, np.float64).reshape(-1, 2)))
        while self._hist and now - self._hist[0][0] > self.cfg["live_hold"]:
            self._hist.popleft()
        self._rebuild()

    def expire(self, now):
        """Drop live scans older than live_hold (call periodically so a dead sensor cannot leave ghosts)."""
        if not self._hist or self._static is None:
            return
        n = len(self._hist)
        while self._hist and now - self._hist[0][0] > self.cfg["live_hold"]:
            self._hist.popleft()
        if len(self._hist) != n:
            self._rebuild()

    def clear_live(self):
        self._hist.clear()
        if self._static is not None:
            self._rebuild()

    def _live_mask(self, grid_shape, res, ox, oy, static_dist):
        if not self._hist:
            return None
        pts = np.vstack([p for _, p in self._hist])
        if len(pts) == 0:
            return None
        h, w = grid_shape
        mx = np.floor((pts[:, 0] - ox) / res).astype(np.int64)
        my = np.floor((pts[:, 1] - oy) / res).astype(np.int64)
        ok = (mx >= 0) & (mx < w) & (my >= 0) & (my < h)
        mx, my = mx[ok], my[ok]
        # live points that sit on the static map ARE the static map (relocalisation noise)
        keep = static_dist[my, mx] > self.cfg["live_ignore_near_static"]
        mask = np.zeros(grid_shape, bool)
        mask[my[keep], mx[keep]] = True
        return mask if mask.any() else None

    def _rebuild(self):
        grid, res, ox, oy, sd = self._static
        mask = self._live_mask(grid.shape, res, ox, oy, sd)
        self.state = Costmap(grid, res, ox, oy, mask, sd, self.cfg)
