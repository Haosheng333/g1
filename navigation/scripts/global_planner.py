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

DANGER_TIER = 66   # costmap tier value of DANGER (g1_slam grid_utils)
LETHAL_TIER = 100

SQRT2 = math.sqrt(2.0)
NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]

DEFAULTS = {
    "cost_scale": 0.10,          # step cost = length * (1 + cost_scale * penalty)
    "heuristic_weight": 1.5,     # >1 = faster, bounded-suboptimal (Python is slow on 0.05 m grids)
    "max_expansions": 400000,
    "time_budget": 3.0,          # seconds
    "start_snap": 0.50,          # metres: move a blocked start to the nearest free cell
    "goal_snap": 0.30,           # metres: a goal that fails the round point-robot clearance but whose real
                                 # footprint fits at the goal heading is reached via the nearest free cell
                                 # within this radius plus a final straight approach (footprint-checked)
    "tight_max_overlap": 0.03,   # metres: a recorded standing pose whose footprint overlaps the MAP by up to
                                 # this much (map noise at furniture edges: the robot physically stood there)
                                 # is still reached, along a straight line back along its own heading
    "tight_max_back": 0.60,      # metres: how far back along that heading free floor must be found
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
    def goal_pose_ok(self, cm, goal_w, goal_yaw):
        """True if the robot can STAND at goal_w facing goal_yaw: in bounds, not lethal or DANGER tier,
        and the 5-circle footprint at that heading keeps `collision_margin` from every lethal cell.
        This is the same yaw-aware check the DWA applies, so a goal close to furniture in front of
        the robot (e.g. a 0.20 m gap to a table) is accepted while a sideways squeeze is not."""
        if goal_yaw is None:
            return False
        mx, my = cm.world_to_map(goal_w[0], goal_w[1])
        if not cm.in_bounds(mx, my) or cm.grid[my, mx] >= DANGER_TIER:
            return False
        return cm.footprint_clearance(goal_w[0], goal_w[1], goal_yaw) >= cm.cfg["collision_margin"]

    def tight_floor(self, cm, goal_w, goal_yaw):
        """Clearance floor for a TIGHT exact-pose goal (footprint overlaps the map slightly at its heading),
        or None if the goal is not a plausible standing pose.  The floor is the goal's own clearance minus
        5 mm: on the final straight approach the robot never gets closer to anything than the pose itself."""
        if goal_yaw is None:
            return None
        mx, my = cm.world_to_map(goal_w[0], goal_w[1])
        if not cm.in_bounds(mx, my) or cm.grid[my, mx] >= LETHAL_TIER:
            return None
        if cm.dist[my, mx] < cm.cfg["circle_radius"]:          # the pelvis point itself is in an obstacle
            return None
        fc = cm.footprint_clearance(goal_w[0], goal_w[1], goal_yaw)
        if fc < -self.cfg["tight_max_overlap"]:
            return None
        return min(fc, cm.cfg["collision_margin"]) - 0.005

    def _tight_entry(self, cm, goal_w, goal_yaw, floor):
        """First FREE cell straight back along goal_yaw from which the straight run to the goal keeps the
        footprint at goal_yaw above `floor`.  -> map cell or None."""
        c, s = math.cos(goal_yaw), math.sin(goal_yaw)
        b = cm.resolution
        while b <= self.cfg["tight_max_back"] + 1e-9:
            p = (goal_w[0] - b * c, goal_w[1] - b * s)
            cell = cm.world_to_map(*p)
            if cm.is_free(*cell):
                a = cm.map_to_world(*cell)
                if self._segment_ok(cm, a, goal_w, goal_yaw, floor):
                    return cell
            b += 0.5 * cm.resolution
        return None

    def _segment_ok(self, cm, a_w, goal_w, goal_yaw, floor):
        dist = math.hypot(goal_w[0] - a_w[0], goal_w[1] - a_w[1])
        for t in np.linspace(0.0, 1.0, max(2, int(dist / (0.5 * cm.resolution)) + 1)):
            if cm.footprint_clearance(a_w[0] + (goal_w[0] - a_w[0]) * t, a_w[1] + (goal_w[1] - a_w[1]) * t,
                                      goal_yaw) < floor:
                return False
        return True

    def standoff(self, cm, goal_w, goal_yaw, max_back=0.40):
        """Nearest pose straight BACK from goal_w along goal_yaw (same heading) that the robot can stand on,
        or None.  For exact-pose goals recorded touching furniture: navigation stops here and a guarded
        straight-ahead step (approach_forward.py) covers the rest with the LIVE front gap.
        -> (x, y, back_distance)"""
        if goal_yaw is None:
            return None
        c, s = math.cos(goal_yaw), math.sin(goal_yaw)
        step = 0.5 * cm.resolution
        b = step
        while b <= max_back + 1e-9:
            p = (goal_w[0] - b * c, goal_w[1] - b * s)
            mx, my = cm.world_to_map(*p)
            if cm.in_bounds(mx, my) and (
                    (cm.is_free(mx, my) and cm.footprint_clearance(p[0], p[1], goal_yaw) >= cm.cfg["collision_margin"])
                    or self.goal_pose_ok(cm, p, goal_yaw)):
                return p[0], p[1], b
            b += step
        return None

    def _approach_ok(self, cm, a_w, goal_w, goal_yaw):
        """The final straight segment a_w -> goal_w is collision-free for the footprint at goal_yaw."""
        dist = math.hypot(goal_w[0] - a_w[0], goal_w[1] - a_w[1])
        for t in np.linspace(0.0, 1.0, max(2, int(dist / (0.5 * cm.resolution)) + 1)):
            x = a_w[0] + (goal_w[0] - a_w[0]) * t
            y = a_w[1] + (goal_w[1] - a_w[1]) * t
            if cm.footprint_clearance(x, y, goal_yaw) < cm.cfg["collision_margin"]:
                return False
        return True

    def plan(self, cm, start_w, goal_w, goal_yaw=None):
        """-> list of (x, y) from `start_w` to `goal_w`, or [] if there is no path.

        `goal_yaw` (optional): when the goal cell fails the point-robot clearance but the footprint
        fits at that heading (goal_pose_ok), A* runs to the nearest free cell within `goal_snap`
        and the path ends with a straight, footprint-checked approach to the exact goal."""
        self.last_info = {"expansions": 0, "reason": ""}
        s = cm.world_to_map(start_w[0], start_w[1])
        g = cm.world_to_map(goal_w[0], goal_w[1])
        approach_from = None
        if not cm.is_free(*g):
            if self.goal_pose_ok(cm, goal_w, goal_yaw):
                approach_from = self._snap_goal(cm, g, goal_w, goal_yaw)
                if approach_from is None:
                    self.last_info["reason"] = ("goal footprint fits at its heading but no free cell within %.2f m "
                                                "has a clear straight approach" % self.cfg["goal_snap"])
                    return []
            else:
                floor = self.tight_floor(cm, goal_w, goal_yaw)
                if floor is None:
                    self.last_info["reason"] = "goal cell is not traversable"
                    return []
                approach_from = self._tight_entry(cm, goal_w, goal_yaw, floor)
                if approach_from is None:
                    self.last_info["reason"] = ("tight goal: no free floor within %.2f m straight back along its "
                                                "heading" % self.cfg["tight_max_back"])
                    return []
                self.last_info["clear_floor"] = floor
            g = approach_from
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
        if approach_from is None:
            pts[-1] = (float(goal_w[0]), float(goal_w[1]))
            pts = self.prune(cm, pts)
            return self.smooth(cm, pts)
        pts = self.smooth(cm, self.prune(cm, pts)) if len(pts) > 1 else pts
        self.last_info["approach_from"] = pts[-1]
        self.last_info["approach_len"] = math.hypot(goal_w[0] - pts[-1][0], goal_w[1] - pts[-1][1])
        tail = self._resample([pts[-1], (float(goal_w[0]), float(goal_w[1]))])
        return pts + tail[1:]

    def _snap_goal(self, cm, cell, goal_w, goal_yaw):
        """Nearest free cell within goal_snap whose straight segment to the goal fits the footprint."""
        r_max = int(math.ceil(self.cfg["goal_snap"] / cm.resolution))
        cands = []
        for dx in range(-r_max, r_max + 1):
            for dy in range(-r_max, r_max + 1):
                d2 = dx * dx + dy * dy
                c = (cell[0] + dx, cell[1] + dy)
                if 0 < d2 <= r_max * r_max and cm.is_free(*c):
                    cands.append((d2, c))
        for _, c in sorted(cands):
            if self._approach_ok(cm, cm.map_to_world(*c), goal_w, goal_yaw):
                return c
        return None

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
