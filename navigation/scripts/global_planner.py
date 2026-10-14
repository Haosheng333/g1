"""Layer (2) of doc section 4.3: the GLOBAL PLANNER.

    A* with tier-aware penalties  ->  line-of-sight pruning  ->  Bezier smoothing  -> path

Pure python/numpy, no rospy.  Works on a `costmap.Costmap` snapshot.

Fixes versus the old planner: a real closed set (the old one overwrote `came_from` on every
pop, which can corrupt the path), penalties from the tier/decay layer, no corner cutting,
a bounded search (expansion cap + time budget), snapping of a start that sits in a blocked cell.
"""
import heapq
import math
import time

import numpy as np

SQRT2 = math.sqrt(2.0)
NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]

DEFAULTS = {
    "cost_scale": 0.10,          # step cost = length * (1 + cost_scale * penalty)
    "heuristic_weight": 1.5,     # >1 = faster, bounded-suboptimal (Python is slow on 0.05 m grids)
    "max_expansions": 400000,
    "time_budget": 3.0,          # seconds
    "start_snap": 0.50,          # metres: move a blocked start to the nearest free cell
    "los_max_penalty": 25.0,     # do not prune across cells costlier than this
    "los_window": 6.0,           # metres of path examined from each anchor when pruning
    "corner_radius": 0.50,       # Bezier rounding radius at each corner (m)
    "resample": 0.10,            # output point spacing (m)
}


class AStarPlanner:
    def __init__(self, **overrides):
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            raise KeyError("unknown planner parameter(s): %s" % sorted(unknown))
        self.cfg = dict(DEFAULTS)
        self.cfg.update(overrides)
        self.last_info = {}

    # ----------------------------------------------------------------- public
    def plan(self, cm, start_w, goal_w):
        """-> list of (x, y) from `start_w` to `goal_w`, or [] if there is no path."""
        self.last_info = {"expansions": 0, "reason": ""}
        s = cm.world_to_map(start_w[0], start_w[1])
        g = cm.world_to_map(goal_w[0], goal_w[1])
        if not cm.is_free(*g):
            self.last_info["reason"] = "goal cell is not traversable"
            return []
        if not cm.is_free(*s):
            s = self._snap(cm, s)
            if s is None:
                self.last_info["reason"] = "start is boxed in"
                return []
        cells = self._astar(cm, s, g)
        if not cells:
            return []
        pts = [cm.map_to_world(mx, my) for mx, my in cells]
        pts[0] = (float(start_w[0]), float(start_w[1]))
        pts[-1] = (float(goal_w[0]), float(goal_w[1]))
        pts = self.prune(cm, pts)
        pts = self.smooth(cm, pts)
        return pts

    # ------------------------------------------------------------------- A*
    def _snap(self, cm, cell):
        r_max = int(math.ceil(self.cfg["start_snap"] / cm.resolution))
        for r in range(1, r_max + 1):
            best = None
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    c = (cell[0] + dx, cell[1] + dy)
                    if cm.is_free(*c):
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, c)
            if best:
                return best[1]
        return None

    def _astar(self, cm, start, goal):
        w, h = cm.width, cm.height
        blocked = cm.blocked.tolist()
        pen = (1.0 + self.cfg["cost_scale"] * cm.penalty).tolist()
        hw = self.cfg["heuristic_weight"]
        gx, gy = goal

        def heur(x, y):
            dx, dy = abs(x - gx), abs(y - gy)
            return hw * ((dx + dy) + (SQRT2 - 2.0) * min(dx, dy))   # octile distance

        g_score = {start: 0.0}
        parent = {start: None}
        closed = set()
        heap = [(heur(*start), 0.0, start)]
        t_end = time.time() + self.cfg["time_budget"]
        expansions = 0
        while heap:
            _, g, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            if g > g_score.get(cur, float("inf")) + 1e-9:
                continue
            closed.add(cur)
            if cur == goal:
                break
            expansions += 1
            if expansions > self.cfg["max_expansions"] or (expansions & 1023 == 0 and time.time() > t_end):
                self.last_info.update(expansions=expansions, reason="search budget exceeded")
                return []
            cx, cy = cur
            for dx, dy, step in NEIGHBOURS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h) or blocked[ny][nx]:
                    continue
                if dx and dy and (blocked[cy][nx] or blocked[ny][cx]):   # no corner cutting
                    continue
                nxt = (nx, ny)
                if nxt in closed:
                    continue
                ng = g + step * 0.5 * (pen[cy][cx] + pen[ny][nx])
                if ng < g_score.get(nxt, float("inf")):
                    g_score[nxt] = ng
                    parent[nxt] = cur
                    heapq.heappush(heap, (ng + heur(nx, ny), ng, nxt))
        self.last_info["expansions"] = expansions
        if goal not in closed:
            self.last_info["reason"] = "no path"
            return []
        out, cur = [], goal
        while cur is not None:
            out.append(cur)
            cur = parent[cur]
        out.reverse()
        return out

    # ---------------------------------------------------------- LOS pruning
    def line_free(self, cm, p, q):
        """True if the straight segment p->q stays traversable and never crosses a costly cell."""
        dist = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(2, int(dist / (0.5 * cm.resolution)) + 1)
        t = np.linspace(0.0, 1.0, n)
        xs = p[0] + (q[0] - p[0]) * t
        ys = p[1] + (q[1] - p[1]) * t
        mx = np.floor((xs - cm.origin_x) / cm.resolution).astype(np.int64)
        my = np.floor((ys - cm.origin_y) / cm.resolution).astype(np.int64)
        if (mx < 0).any() or (mx >= cm.width).any() or (my < 0).any() or (my >= cm.height).any():
            return False
        if cm.blocked[my, mx].any():
            return False
        return float(cm.penalty[my, mx].max()) <= self.cfg["los_max_penalty"]

    def prune(self, cm, pts):
        """Greedy farthest-visible-point pruning of the raw grid path."""
        if len(pts) <= 2:
            return pts
        seg = [0.0]
        for a, b in zip(pts[:-1], pts[1:]):
            seg.append(seg[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        out, i, n = [pts[0]], 0, len(pts)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and (seg[j] - seg[i] > self.cfg["los_window"] or not self.line_free(cm, pts[i], pts[j])):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    # -------------------------------------------------------- Bezier smoothing
    @staticmethod
    def _bezier(a, p, b, m):
        t = np.linspace(0.0, 1.0, m)[:, None]
        a, p, b = np.array(a), np.array(p), np.array(b)
        return (1 - t) ** 2 * a + 2 * (1 - t) * t * p + t ** 2 * b

    def smooth(self, cm, pts):
        """Round every interior corner with a quadratic Bezier; fall back to the sharp corner
        if the rounded curve would touch a blocked cell.  Output is resampled uniformly."""
        if len(pts) <= 2:
            return self._resample(pts)
        out = [pts[0]]
        for k in range(1, len(pts) - 1):
            p_prev, p, p_next = np.array(pts[k - 1]), np.array(pts[k]), np.array(pts[k + 1])
            l1, l2 = np.linalg.norm(p - p_prev), np.linalg.norm(p_next - p)
            if l1 < 1e-6 or l2 < 1e-6:
                out.append(tuple(p))
                continue
            r = min(self.cfg["corner_radius"], 0.45 * l1, 0.45 * l2)
            curve = None
            for _ in range(4):
                a = p + (p_prev - p) / l1 * r
                b = p + (p_next - p) / l2 * r
                cand = self._bezier(a, p, b, max(5, int(2.0 * r / cm.resolution)))
                if self._curve_ok(cm, cand):
                    curve = cand
                    break
                r *= 0.5
            if curve is None:
                out.append(tuple(p))
            else:
                out.extend((float(x), float(y)) for x, y in curve)
        out.append(pts[-1])
        return self._resample(out)

    @staticmethod
    def _curve_ok(cm, curve):
        mx = np.floor((curve[:, 0] - cm.origin_x) / cm.resolution).astype(np.int64)
        my = np.floor((curve[:, 1] - cm.origin_y) / cm.resolution).astype(np.int64)
        if (mx < 0).any() or (mx >= cm.width).any() or (my < 0).any() or (my >= cm.height).any():
            return False
        return not cm.blocked[my, mx].any()

    def _resample(self, pts):
        ds = self.cfg["resample"]
        arr = np.array(pts, dtype=np.float64)
        d = np.r_[0.0, np.cumsum(np.hypot(*np.diff(arr, axis=0).T))]
        total = d[-1]
        if total < 1e-6:
            return [tuple(arr[0]), tuple(arr[-1])]
        s = np.arange(0.0, total, ds)
        s = np.r_[s, total]
        x = np.interp(s, d, arr[:, 0])
        y = np.interp(s, d, arr[:, 1])
        return [(float(a), float(b)) for a, b in zip(x, y)]
