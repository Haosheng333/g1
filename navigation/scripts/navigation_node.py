#!/usr/bin/env python3
"""navigation_node - ONE node, three classic layers (doc section 4.3) + localization gate (4.2).

    /map (tiered) + /obstacle_cloud (live)  ->  costmap  ->  A* global planner  ->  holonomic DWA
                                                                     |                    |
                                                         /navigation/global_path   /navigation/cmd_vel

Interfaces
  sub  /map                      nav_msgs/OccupancyGrid     tiered grid from g1_slam/grid_composer (~1 Hz)
  sub  /obstacle_cloud           sensor_msgs/PointCloud2    live lethal layer, frame = global frame (~10 Hz)
  sub  /move_base_simple/goal    geometry_msgs/PoseStamped  rviz "2D Nav Goal"
  sub  /slam/manager/state       std_msgs/Bool              latched localized flag
  sub  /navigation/cancel        std_msgs/Empty             abort the active goal (-> FAILURE, robot stops)
  srv  /navigation/navigate      g1_navigation/NavigateToObject
  pub  /navigation/status        std_msgs/String            PLANNING|FOLLOWING|RECOVERING|SUCCESS|FAILURE
  pub  /navigation/global_path   nav_msgs/Path              (latched)
  pub  /navigation/local_path    nav_msgs/Path              best DWA trajectory (rviz)
  pub  /navigation/costmap       nav_msgs/OccupancyGrid     combined cost picture (rviz, ~1 Hz)
  pub  /navigation/cmd_vel       geometry_msgs/Twist        vx, vy, wz -> cmd_vel_gate -> /locomotion/cmd_vel
  pub  /navigation/internal_state std_msgs/String           debug

Status rules (the orchestrator's saw_active guard depends on them): the status is re-published every
tick, terminal states persist until the next goal, accepting a goal publishes PLANNING immediately,
goals are SUSPENDED (not rejected) while un-localized.
"""
import math
import threading
import traceback

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty, String

from g1_navigation.srv import NavigateToObject, NavigateToObjectResponse
from g1_slam import cloud_utils as cu

import os, sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # noqa: E402
from costmap import CostmapManager
from global_planner import AStarPlanner
from local_planner import SPEC_XY_TOL, SPEC_YAW_TOL, DwaController
from semantic_goal import find_approach_pose, load_semantic, lookup_object

PLANNING, FOLLOWING, RECOVERING, SUCCESS, FAILURE = (
    "PLANNING", "FOLLOWING", "RECOVERING", "SUCCESS", "FAILURE")

TF_ERRORS = (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class NavigationNode:
    def __init__(self):
        rospy.init_node("navigation_node")
        p = rospy.get_param
        self.global_frame = p("~global_frame", "map")
        self.robot_frame = p("~robot_frame", "pelvis")
        self.semantic_file = p("~semantic_file", "")
        self.default_approach = float(p("~default_approach_distance", 0.6))
        self.use_gate = bool(p("~use_localization_gate", True))
        self.still_time = float(p("~still_time", 3.0))
        self.sweep_amp = float(p("~sweep_amplitude", 0.52))
        self.sweep_speed = float(p("~sweep_speed", 0.4))
        self.gate_timeout = float(p("~gate_timeout", 60.0))
        self.relocalize_without_goal = bool(p("~relocalize_without_goal", True))
        self.plan_timeout = float(p("~plan_timeout", 15.0))
        self.max_recoveries = int(p("~max_recoveries", 3))
        self.recovery_wait = float(p("~recovery_wait", 1.0))
        self.stuck_window = float(p("~stuck_window", 10.0))
        self.stuck_dist = float(p("~stuck_dist", 0.05))
        self.blocked_timeout = float(p("~blocked_timeout", 2.0))
        self.replan_min_interval = float(p("~replan_min_interval", 1.0))
        self.path_check_distance = float(p("~path_check_distance", 2.0))
        self.live_period = 1.0 / float(p("~live_rate", 5.0))
        self.spec_settle = float(p("~spec_settle_time", 2.0))
        self.rate_hz = float(p("~rate", 10.0))

        self.cmgr = CostmapManager(**dict(p("~costmap", {})))
        self.planner = AStarPlanner(**dict(p("~global_planner", {})))
        self.dwa = DwaController(**dict(p("~dwa", {})))
        self.tf_listener = tf.TransformListener()

        self.lock = threading.RLock()
        self.goal = None
        self.status = None
        self.internal = "IDLE"
        self.localized = not self.use_gate
        self.gate_t0 = None
        self.gate_expired = False
        self.recover_until = None
        self.cur_vel = (0.0, 0.0, 0.0)
        self._moving = False
        self._last_map_key = None
        self._last_live_t = 0.0
        self._last_costmap_pub = 0.0
        self._last_state_pub = None

        self.cmd_pub = rospy.Publisher("/navigation/cmd_vel", Twist, queue_size=10)
        self.path_pub = rospy.Publisher("/navigation/global_path", Path, queue_size=1, latch=True)
        self.local_pub = rospy.Publisher("/navigation/local_path", Path, queue_size=1)
        self.costmap_pub = rospy.Publisher("/navigation/costmap", OccupancyGrid, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/navigation/status", String, queue_size=10)
        self.internal_pub = rospy.Publisher("/navigation/internal_state", String, queue_size=10)

        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/obstacle_cloud", PointCloud2, self.obstacle_cb, queue_size=1, buff_size=2 ** 22)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("/navigation/cancel", Empty, self.cancel_cb, queue_size=1)
        if self.use_gate:
            rospy.Subscriber("/slam/manager/state", Bool, self.state_cb, queue_size=1)
        rospy.Service("/navigation/navigate", NavigateToObject, self.navigate_cb)
        rospy.on_shutdown(self._stop)
        rospy.loginfo("[navigation_node] frame=%s robot=%s gate=%s semantic=%s",
                      self.global_frame, self.robot_frame, self.use_gate, self.semantic_file or "-")

    # ------------------------------------------------------------ callbacks
    def map_cb(self, msg):
        key = (msg.info.width, msg.info.height, msg.info.resolution,
               msg.info.origin.position.x, msg.info.origin.position.y, msg.data)
        if key == self._last_map_key:
            return                               # /map is re-published at 1 Hz; skip identical grids
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.cmgr.set_static(grid, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)
        self._last_map_key = key
        rospy.loginfo("[navigation_node] map updated: %dx%d @ %.2f m", msg.info.width, msg.info.height,
                      msg.info.resolution)

    def obstacle_cb(self, msg):
        now = rospy.Time.now().to_sec()
        if now - self._last_live_t < self.live_period:
            return
        if msg.header.frame_id != self.global_frame:
            rospy.logerr_throttle(5.0, "[navigation_node] /obstacle_cloud is in '%s', expected '%s': ignored",
                                  msg.header.frame_id, self.global_frame)
            return
        try:
            pts = cu.pc2_to_xyz(msg)
        except ValueError as exc:
            rospy.logerr_throttle(5.0, "[navigation_node] bad /obstacle_cloud: %s", exc)
            return
        self._last_live_t = now
        self.cmgr.set_live_points(pts[:, :2], now)

    def state_cb(self, msg):
        with self.lock:
            was, self.localized = self.localized, bool(msg.data)
        if self.localized and not was:
            rospy.loginfo("[navigation_node] localized: gate open")
        elif was and not self.localized:
            rospy.logwarn("[navigation_node] LOCALIZATION LOST: goal suspended")

    def goal_cb(self, msg):
        rospy.loginfo("[navigation_node] rviz goal in frame '%s'", msg.header.frame_id)
        self._accept({"kind": "pose", "stamped": msg})

    def cancel_cb(self, _msg):
        with self.lock:
            goal = self.goal
        if goal is not None:
            self._finish(goal, FAILURE, "cancelled via /navigation/cancel")

    def navigate_cb(self, req):
        name = req.class_name.strip()
        if not name:
            return NavigateToObjectResponse(success=False, message="class_name is empty")
        try:
            semantic = load_semantic(self.semantic_file)
        except Exception as exc:  # noqa: BLE001
            return NavigateToObjectResponse(
                success=False, message="cannot read semantic file '%s': %s" % (self.semantic_file, exc))
        obj = lookup_object(semantic, name)
        if obj is None:
            return NavigateToObjectResponse(
                success=False,
                message="unknown class '%s'. Known: %s" % (name, ", ".join(sorted(semantic)) or "(none)"))
        approach = req.approach_distance if req.approach_distance > 0 else self.default_approach
        self._accept({"kind": "semantic", "name": name, "object": obj, "approach": float(approach)})
        msg = "accepted '%s' (approach %.2f m)" % (name, approach)
        if self.use_gate and not self.localized:
            msg += "; suspended until localized"
        return NavigateToObjectResponse(success=True, message=msg)

    def _accept(self, goal):
        goal.update({"path": None, "target": None, "recoveries": 0})
        with self.lock:
            self.goal = goal
            self.gate_t0 = None
            self.gate_expired = False
            self.recover_until = None
            self.dwa.reset()
            self._set_status(PLANNING)          # published BEFORE the service call returns

    # --------------------------------------------------------------- status
    def _set_status(self, state):
        self.status = state
        self.status_pub.publish(String(data=state))

    def _set_status_if_changed(self, state):
        if self.status != state:
            self._set_status(state)

    def _finish(self, goal, state, why):
        with self.lock:
            if self.goal is not goal:            # a newer goal replaced this one meanwhile
                return
            self.goal = None
            self.recover_until = None
            self._set_status(state)
        (rospy.loginfo if state == SUCCESS else rospy.logwarn)("[navigation_node] %s: %s", state, why)
        self._stop()

    # ------------------------------------------------------------ utilities
    def _publish_cmd(self, vx, vy, wz):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        self.cmd_pub.publish(cmd)
        self.cur_vel = (vx, vy, wz)
        self._moving = any(abs(v) > 1e-6 for v in (vx, vy, wz))

    def _stop(self):
        if self._moving:
            self.cmd_pub.publish(Twist())
            self._moving = False
        self.cur_vel = (0.0, 0.0, 0.0)

    def get_robot_pose(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(self.global_frame, self.robot_frame, rospy.Time(0))
        except TF_ERRORS:
            return None
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return (trans[0], trans[1], yaw)

    def _pose_in_global(self, stamped):
        ps = PoseStamped()
        ps.header = stamped.header
        ps.pose = stamped.pose
        fid = ps.header.frame_id.lstrip("/")
        if fid not in ("", self.global_frame):
            ps.header.stamp = rospy.Time(0)      # latest available transform
            ps = self.tf_listener.transformPose(self.global_frame, ps)
        q = ps.pose.orientation
        if q.x == q.y == q.z == q.w == 0.0:
            yaw = None
        else:
            yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (ps.pose.position.x, ps.pose.position.y, yaw)

    def _make_path(self, pts):
        msg = Path()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = rospy.Time.now()
        for x, y in pts:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _publish_costmap(self, now):
        cm = self.cmgr.state
        if cm is None or now - self._last_costmap_pub < 1.0 or cm is self._last_state_pub:
            return
        msg = OccupancyGrid()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution = cm.resolution
        msg.info.width, msg.info.height = cm.width, cm.height
        msg.info.origin.position.x, msg.info.origin.position.y = cm.origin_x, cm.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = cm.to_occupancy_data().flatten().tolist()
        self.costmap_pub.publish(msg)
        self._last_costmap_pub, self._last_state_pub = now, cm

    # ----------------------------------------------------- relocalizing gate
    def _sweep_wz(self, t):
        """Open-loop bounded sweep: +amp, then -2*amp, +2*amp, ... at sweep_speed."""
        if t < 0.0:
            return 0.0
        first = self.sweep_amp / self.sweep_speed
        if t < first:
            return self.sweep_speed
        k = int((t - first) // (2.0 * self.sweep_amp / self.sweep_speed))
        return -self.sweep_speed if k % 2 == 0 else self.sweep_speed

    def _gate_tick(self, now, goal):
        if goal is not None:
            goal["path"] = None                  # the map->odom transform may jump: re-plan afterwards
            goal.pop("plan_t0", None)
        if self.gate_t0 is None:
            self.gate_t0, self.gate_expired = now, False
            rospy.logwarn("[navigation_node] RELOCALIZING (still %.0f s, then sweep +-%.2f rad, "
                          "timeout %.0f s)", self.still_time, self.sweep_amp, self.gate_timeout)
        if goal is None and not self.relocalize_without_goal:
            self._stop()
            self.internal = "WAITING_FOR_LOCALIZATION"
            return
        elapsed = now - self.gate_t0
        if elapsed >= self.gate_timeout:
            self._stop()
            self.internal = "RELOCALIZATION_FAILED"
            if not self.gate_expired:
                self.gate_expired = True
                rospy.logerr("[navigation_node] not localized after %.0f s", self.gate_timeout)
                if goal is not None:
                    self._finish(goal, FAILURE, "not localized within %.0f s" % self.gate_timeout)
            return
        self.internal = "RELOCALIZING"
        if goal is not None:
            self._set_status_if_changed(PLANNING)   # suspended, still an ACTIVE state
        self._publish_cmd(0.0, 0.0, self._sweep_wz(elapsed - self.still_time))

    def _leave_gate(self):
        self.gate_t0, self.gate_expired = None, False
        self._stop()

    # ---------------------------------------------------------- state machine
    def tick(self):
        now = rospy.Time.now().to_sec()
        self.cmgr.expire(now)
        with self.lock:
            goal, localized = self.goal, self.localized
        if not localized:
            self._gate_tick(now, goal)
            return
        if self.gate_t0 is not None:
            self._leave_gate()
        if goal is None:
            self.internal = "IDLE"
            return
        if self.recover_until is not None:
            if now < self.recover_until:
                self._stop()
                self.internal = "RECOVERING"
                return
            self.recover_until, goal["path"] = None, None
        if goal["path"] is None:
            self._plan(goal, now)
        else:
            self._follow(goal, now)

    def _recover(self, goal, now, why):
        goal["recoveries"] += 1
        if goal["recoveries"] > self.max_recoveries:
            self._finish(goal, FAILURE, why)
            return
        rospy.logwarn("[navigation_node] RECOVERING (%d/%d): %s", goal["recoveries"], self.max_recoveries, why)
        self._stop()
        goal["path"] = None
        self.recover_until = now + self.recovery_wait
        self._set_status_if_changed(RECOVERING)

    def _plan(self, goal, now):
        self.internal = "PLANNING"
        goal.setdefault("plan_t0", now)
        pose, cm = self.get_robot_pose(), self.cmgr.state
        target, why = None, ""
        if pose is not None and cm is not None:
            try:
                if goal["kind"] == "semantic":
                    obj = goal["object"]
                    target = find_approach_pose(cm, self.planner, (obj["x"], obj["y"]), goal["approach"],
                                                (pose[0], pose[1]), approach_yaw=obj["approach_yaw"])
                    why = "no standable, reachable approach pose near '%s'" % goal["name"]
                else:
                    target = self._pose_in_global(goal["stamped"])
                    why = "goal is not a free cell"
            except TF_ERRORS:
                pose = None                      # goal frame not resolvable yet: treat like missing pose
        if pose is None or cm is None:
            if now - goal["plan_t0"] > self.plan_timeout:
                self._finish(goal, FAILURE, "no robot pose (%s->%s) or no /map after %.0f s"
                             % (self.global_frame, self.robot_frame, self.plan_timeout))
            else:
                self._set_status_if_changed(PLANNING)
            return
        if target is None:
            self._finish(goal, FAILURE, why)
            return
        path = self.planner.plan(cm, (pose[0], pose[1]), (target[0], target[1]))
        if not path:
            self._recover(goal, now, "planner found no path (%s)" % self.planner.last_info.get("reason", ""))
            return
        path[-1] = (target[0], target[1])        # end exactly on the goal, not the cell centre
        goal["path"], goal["target"] = path, target
        goal["progress"] = (now, pose[0], pose[1], pose[2])
        goal["blocked_since"] = None
        goal["spec_since"] = None
        goal["last_plan"] = now
        self.dwa.reset()
        self.path_pub.publish(self._make_path(path))
        rospy.loginfo("[navigation_node] path with %d points to (%.2f, %.2f, yaw %s)", len(path),
                      target[0], target[1], "any" if target[2] is None else "%.2f" % target[2])
        self._set_status_if_changed(FOLLOWING)

    def _path_blocked(self, cm, path, pose):
        """True if the next `path_check_distance` metres of path (beyond 0.4 m) are no longer traversable."""
        arr = np.asarray(path, dtype=np.float64)
        i = int(np.argmin(np.hypot(arr[:, 0] - pose[0], arr[:, 1] - pose[1])))
        acc = 0.0
        for j in range(i, len(arr)):
            if j > i:
                acc += float(np.hypot(*(arr[j] - arr[j - 1])))
            if acc > self.path_check_distance:
                break
            if math.hypot(arr[j, 0] - pose[0], arr[j, 1] - pose[1]) < 0.4:
                continue
            if not cm.is_free(*cm.world_to_map(arr[j, 0], arr[j, 1])):
                return True
        return False

    def _follow(self, goal, now):
        self.internal = "FOLLOWING"
        pose, cm = self.get_robot_pose(), self.cmgr.state
        if pose is None or cm is None:
            self._stop()
            return
        # live obstacle appeared on the path: re-plan (rate-limited)
        if now - goal["last_plan"] > self.replan_min_interval and self._path_blocked(cm, goal["path"], pose):
            rospy.logwarn("[navigation_node] path blocked by a new obstacle: re-planning")
            goal["path"] = None
            self._stop()
            return
        gx, gy, gyaw = goal["target"]
        res = self.dwa.compute(pose, self.cur_vel, goal["path"], (gx, gy), gyaw, cm)
        self.internal = "FOLLOWING/" + res.phase
        # Safety net: the controller aims at 0.08 m / 0.08 rad, tighter than the spec.  If the walking
        # controller's dead-band keeps it from getting there but it has been inside the SPEC tolerance
        # (doc section 7: < 0.10 m AND < 0.1 rad) for `spec_settle_time`, that IS arrival.
        d_goal = math.hypot(gx - pose[0], gy - pose[1])
        yaw_err = abs(_wrap(gyaw - pose[2])) if gyaw is not None else 0.0
        if d_goal < SPEC_XY_TOL and yaw_err < SPEC_YAW_TOL:
            if goal.get("spec_since") is None:
                goal["spec_since"] = now
            elif now - goal["spec_since"] >= self.spec_settle:
                self._finish(goal, SUCCESS, "within spec %.3f m / %.3f rad for %.1f s"
                             % (d_goal, yaw_err, self.spec_settle))
                return
        else:
            goal["spec_since"] = None
        if res.reached:
            self._finish(goal, SUCCESS, "within %.2f m / %.1f rad (spec %.2f / %.1f)"
                         % (math.hypot(gx - pose[0], gy - pose[1]),
                            abs(_wrap(gyaw - pose[2])) if gyaw is not None else 0.0, SPEC_XY_TOL, SPEC_YAW_TOL))
            return
        if res.blocked:                          # no collision-free trajectory
            self._stop()
            if goal.get("blocked_since") is None:
                goal["blocked_since"] = now
            elif now - goal["blocked_since"] > self.blocked_timeout:
                self._recover(goal, now, "no collision-free motion for %.1f s" % self.blocked_timeout)
            return
        goal["blocked_since"] = None
        t_ref, x_ref, y_ref, yaw_ref = goal["progress"]
        if (math.hypot(pose[0] - x_ref, pose[1] - y_ref) > self.stuck_dist
                or abs(_wrap(pose[2] - yaw_ref)) > 0.05):
            goal["progress"] = (now, pose[0], pose[1], pose[2])
        elif now - t_ref > self.stuck_window:
            self._recover(goal, now, "no progress for %.0f s" % self.stuck_window)
            return
        self._set_status_if_changed(FOLLOWING)
        self._publish_cmd(res.vx, res.vy, res.wz)
        if res.traj is not None:
            self.local_pub.publish(self._make_path(res.traj))

    # ------------------------------------------------------------------ loop
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self.tick()
                self._publish_costmap(rospy.Time.now().to_sec())
            except Exception:  # noqa: BLE001 - never leave the orchestrator waiting forever
                rospy.logerr("[navigation_node] tick failed:\n%s", traceback.format_exc())
                with self.lock:
                    goal = self.goal
                if goal is not None:
                    self._finish(goal, FAILURE, "internal error")
            if self.status is not None:
                self.status_pub.publish(String(data=self.status))   # re-publish every tick
            self.internal_pub.publish(String(data=self.internal))
            rate.sleep()


if __name__ == "__main__":
    try:
        NavigationNode().run()
    except rospy.ROSInterruptException:
        pass
