"""Layer (3) of doc section 4.3: the LOCAL PLANNER - holonomic DWA with three phases.

    NAVIGATION  far from the goal: follow the global path (look-ahead target), face the travel direction
    APPROACH    closer than `approach_dist`: slower, blend the heading towards the goal yaw
    TERMINAL    closer than `terminal_dist`: very slow, fine position, then rotate to the goal yaw

Every sampled constant-velocity trajectory is rolled out over `horizon` seconds and checked with the
5-circle footprint against the costmap (distance transform); colliding trajectories are discarded.
Arrival (doc section 7 spec: < 0.10 m AND < 0.1 rad); the controller declares `reached` at the tighter
xy_tol / yaw_tol below (0.08 / 0.08) to keep a margin.

Pure numpy, no rospy.  Velocities are in the BODY frame: vx forward, vy left, wz counter-clockwise.
"""
import math

import numpy as np

NAVIGATION, APPROACH, TERMINAL = "NAVIGATION", "APPROACH", "TERMINAL"

SPEC_XY_TOL = 0.10    # doc section 7: navigation arrival tolerance
SPEC_YAW_TOL = 0.1

DEFAULTS = {
    # (max_vx, max_vy, max_wz) per phase -- conservative for a walking humanoid; tune with the robot on a stand
    "nav_limits": (0.30, 0.15, 0.60),
    "approach_limits": (0.15, 0.10, 0.40),
    "terminal_limits": (0.06, 0.06, 0.30),
    "min_vx": -0.10,
    "acc": (0.6, 0.4, 1.5),          # accel limits (m/s^2, m/s^2, rad/s^2)
    "control_dt": 0.1,               # period between compute() calls
    "sim_dt": 0.1,
    "horizon": 1.6,
    "n_vx": 7, "n_vy": 5, "n_wz": 9,
    "lookahead": 0.8,
    "approach_dist": 0.8,
    "terminal_dist": 0.30,
    # The controller aims tighter than the document's arrival spec (0.10 m / 0.1 rad) so that pose
    # noise and walking-controller dead-band do not turn a "just inside" stop into a miss.
    "xy_tol": 0.08,
    "yaw_tol": 0.08,
    "w_goal": 3.0, "w_head": 1.5, "w_clear": 2.0, "w_speed": 0.5, "w_lateral": 0.8, "w_smooth": 0.1,
    # stay on the global path (keeps the body centred in narrow gaps instead of cutting corners):
    "w_path": 3.0,
    "clear_pref": 0.30,              # prefer at least this footprint clearance (m)
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DwaResult:
    __slots__ = ("vx", "vy", "wz", "phase", "reached", "blocked", "traj", "cost")

    def __init__(self, vx=0.0, vy=0.0, wz=0.0, phase=NAVIGATION, reached=False, blocked=False, traj=None, cost=0.0):
        self.vx, self.vy, self.wz = vx, vy, wz
        self.phase, self.reached, self.blocked = phase, reached, blocked
        self.traj, self.cost = traj, cost


class DwaController:
    def __init__(self, **overrides):
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            raise KeyError("unknown DWA parameter(s): %s" % sorted(unknown))
        self.cfg = dict(DEFAULTS)
        self.cfg.update(overrides)
        self.phase = NAVIGATION

    def reset(self):
        self.phase = NAVIGATION

    # ---------------------------------------------------------------- helpers
    def _update_phase(self, d):
        c = self.cfg
        if self.phase == NAVIGATION and d < c["approach_dist"]:
            self.phase = APPROACH
        if self.phase == APPROACH and d < c["terminal_dist"]:
            self.phase = TERMINAL
        if self.phase == TERMINAL and d > c["terminal_dist"] * 1.5:
            self.phase = APPROACH
        if self.phase == APPROACH and d > c["approach_dist"] * 1.3:
            self.phase = NAVIGATION

    @staticmethod
    def _lookahead(path, x, y, length):
        arr = np.asarray(path, dtype=np.float64)
        i = int(np.argmin(np.hypot(arr[:, 0] - x, arr[:, 1] - y)))
        acc, j = 0.0, i
        while j < len(arr) - 1 and acc < length:
            acc += float(np.hypot(*(arr[j + 1] - arr[j])))
            j += 1
        return float(arr[j, 0]), float(arr[j, 1])

    @staticmethod
    def _path_deviation(path, x, y, X, Y):
        """Mean distance of each trajectory's middle and end point to the global path (m)."""
        arr = np.asarray(path, dtype=np.float64)
        i = int(np.argmin(np.hypot(arr[:, 0] - x, arr[:, 1] - y)))
        win = arr[max(0, i - 5):i + 60]                       # ~6 m of path ahead (points are ~0.1 m apart)
        out = 0.0
        for col in (X.shape[1] // 2, X.shape[1] - 1):
            dx = X[:, col][:, None] - win[None, :, 0]
            dy = Y[:, col][:, None] - win[None, :, 1]
            out = out + np.sqrt((dx * dx + dy * dy).min(axis=1))
        return out / 2.0

    def _axis(self, cur, lo_lim, hi_lim, acc, n):
        dt = self.cfg["control_dt"]
        lo, hi = max(lo_lim, cur - acc * dt), min(hi_lim, cur + acc * dt)
        if lo > hi:
            lo = hi = max(lo_lim, min(hi_lim, cur))
        vals = np.linspace(lo, hi, n) if hi - lo > 1e-9 else np.array([lo])
        if lo <= 0.0 <= hi:
            vals = np.unique(np.append(vals, 0.0))
        return vals

    # ---------------------------------------------------------------- main
    def compute(self, pose, cur_vel, path, goal_xy, goal_yaw, cm, clear_floor=None, xy_tol=None):
        """pose=(x, y, yaw) in the map frame, cur_vel=(vx, vy, wz) last commanded body velocity,
        path = list of (x, y) (may be empty), goal_xy=(x, y), goal_yaw=float|None, cm=costmap.Costmap.
        clear_floor: lower collision limit used ONLY on the final straight approach to a tight goal
        (navigation_node passes it there; the planner derived it from the goal's own clearance).
        xy_tol: arrival radius override (tight goals use a smaller one than cfg xy_tol)."""
        c = self.cfg
        tol_xy = c["xy_tol"] if xy_tol is None else float(xy_tol)
        x, y, yaw = pose
        gx, gy = goal_xy
        dist_goal = math.hypot(gx - x, gy - y)
        yaw_err = wrap(goal_yaw - yaw) if goal_yaw is not None else 0.0

        if dist_goal < tol_xy and abs(yaw_err) < c["yaw_tol"]:
            return DwaResult(phase=self.phase, reached=True)

        self._update_phase(dist_goal)
        phase = self.phase
        lim_vx, lim_vy, lim_wz = c[{NAVIGATION: "nav_limits", APPROACH: "approach_limits",
                                    TERMINAL: "terminal_limits"}[phase]]

        # target point and desired heading --------------------------------------------------
        if phase == NAVIGATION and path:
            tx, ty = self._lookahead(path, x, y, c["lookahead"])
        else:
            tx, ty = gx, gy
        travel = math.atan2(ty - y, tx - x) if math.hypot(tx - x, ty - y) > 0.05 else yaw
        if goal_yaw is None:
            desired = travel if phase != TERMINAL else yaw
        elif phase == NAVIGATION:
            desired = travel
        elif phase == APPROACH:
            blend = min(1.0, max(0.0, (c["approach_dist"] - dist_goal) / (c["approach_dist"] - c["terminal_dist"])))
            desired = wrap(travel + blend * wrap(goal_yaw - travel))
        else:
            desired = goal_yaw

        # dynamic window samples ------------------------------------------------------------
        ax, ay, aw = c["acc"]
        cvx, cvy, cwz = cur_vel
        if phase == TERMINAL and dist_goal < tol_xy:
            vx_s, vy_s = np.array([0.0]), np.array([0.0])          # position done: only rotate
        else:
            vx_s = self._axis(cvx, c["min_vx"], lim_vx, ax, c["n_vx"])
            vy_s = self._axis(cvy, -lim_vy, lim_vy, ay, c["n_vy"])
        wz_s = self._axis(cwz, -lim_wz, lim_wz, aw, c["n_wz"])
        VX, VY, WZ = [a.ravel() for a in np.meshgrid(vx_s, vy_s, wz_s, indexing="ij")]

        # roll out all trajectories at once --------------------------------------------------
        dt = c["sim_dt"]
        steps = max(1, int(round(c["horizon"] / dt)))
        k = np.arange(steps)
        th_mid = yaw + WZ[:, None] * ((k + 0.5) * dt)[None, :]
        dx = (VX[:, None] * np.cos(th_mid) - VY[:, None] * np.sin(th_mid)) * dt
        dy = (VX[:, None] * np.sin(th_mid) + VY[:, None] * np.cos(th_mid)) * dt
        X = x + np.cumsum(dx, axis=1)
        Y = y + np.cumsum(dy, axis=1)
        TH = yaw + WZ[:, None] * ((k + 1) * dt)[None, :]
        min_clear = cm.footprint_clearance_many(X, Y, TH).min(axis=1)

        cur_clear = cm.footprint_clearance(x, y, yaw)
        thr = cm.cfg["collision_margin"]
        if clear_floor is not None:
            thr = min(thr, clear_floor)
        if cur_clear < thr:                       # already too close: only allow not getting worse
            thr = cur_clear - 0.005
        ok = min_clear >= thr
        if not ok.any():
            return DwaResult(phase=phase, blocked=True)

        # score -------------------------------------------------------------------------------
        goal_cost = np.hypot(X[:, -1] - tx, Y[:, -1] - ty)
        path_cost = 0.0
        if phase != TERMINAL and path and c["w_path"] > 0.0:
            path_cost = c["w_path"] * self._path_deviation(path, x, y, X, Y)
        head_cost = np.abs(np.arctan2(np.sin(TH[:, -1] - desired), np.cos(TH[:, -1] - desired)))
        clear_cost = np.maximum(0.0, c["clear_pref"] - min_clear)
        smooth = (np.abs(VX - cvx) / max(lim_vx, 1e-6) + np.abs(VY - cvy) / max(lim_vy, 1e-6)
                  + np.abs(WZ - cwz) / max(lim_wz, 1e-6))
        w_head = c["w_head"] * (3.0 if phase == TERMINAL else 1.0)
        cost = (c["w_goal"] * goal_cost + w_head * head_cost + c["w_clear"] * clear_cost
                + c["w_smooth"] * smooth + path_cost)
        if phase == NAVIGATION:
            cost = cost + c["w_speed"] * (lim_vx - VX) / lim_vx + c["w_lateral"] * np.abs(VY) / max(lim_vy, 1e-6)
        cost = np.where(ok, cost, np.inf)
        b = int(np.argmin(cost))
        return DwaResult(float(VX[b]), float(VY[b]), float(WZ[b]), phase, False, False,
                         np.stack([X[b], Y[b]], axis=1), float(cost[b]))
