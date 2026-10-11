#!/usr/bin/env python3
"""navigation_node - goal interfaces, status, localization gate (doc section 4.2 / 4.3).

Interfaces
  sub  /map                      nav_msgs/OccupancyGrid   tiered grid from g1_slam/grid_composer
  sub  /move_base_simple/goal    geometry_msgs/PoseStamped  rviz "2D Nav Goal"
  sub  /slam/manager/state       std_msgs/Bool             latched localized flag
  sub  /navigation/cancel        std_msgs/Empty            abort the active goal (-> FAILURE, robot stops)
  srv  /navigation/navigate      g1_navigation/NavigateToObject  semantic goal
  pub  /navigation/status        std_msgs/String           PLANNING|FOLLOWING|RECOVERING|SUCCESS|FAILURE
  pub  /navigation/global_path   nav_msgs/Path             (latched)
  pub  /navigation/cmd_vel       geometry_msgs/Twist       -> cmd_vel_gate -> /locomotion/cmd_vel
  pub  /navigation/internal_state std_msgs/String          debug: IDLE|RELOCALIZING|...

Status rules (the orchestrator's saw_active guard depends on them):
  * the current status is re-published every tick; a terminal state (SUCCESS /
    FAILURE) stays on the topic until the next goal arrives
  * accepting a goal immediately publishes PLANNING (before the service returns)
  * goals that arrive while un-localized are SUSPENDED (status stays PLANNING),
    never rejected; the node sweeps in place (3 s still, then +-0.52 rad at
    0.4 rad/s) for at most 60 s and reports FAILURE only if localization never comes.
"""
import math
import threading
import traceback

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, Empty, String

from g1_navigation.srv import NavigateToObject, NavigateToObjectResponse

from costmap import OccupancyCostmap
from global_planner import AStarPlanner
from local_planner import PurePursuitController, XY_TOL, YAW_TOL
from semantic_goal import find_approach_pose, load_semantic, lookup_object

PLANNING, FOLLOWING, RECOVERING, SUCCESS, FAILURE = (
    "PLANNING", "FOLLOWING", "RECOVERING", "SUCCESS", "FAILURE")

TF_ERRORS = (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)


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
        self.inflation_cells = int(p("~inflation_cells", 0))  # tiers already encode clearance
        self.rate_hz = float(p("~rate", 10.0))

        self.costmap = None
        self._last_map_key = None
        self.planner = AStarPlanner()
        self.controller = PurePursuitController(xy_tol=XY_TOL, yaw_tol=YAW_TOL)
        self.tf_listener = tf.TransformListener()

        self.lock = threading.RLock()
        self.goal = None
        self.status = None            # last published status string (None until first goal)
        self.internal = "IDLE"
        self.localized = not self.use_gate
        self.gate_t0 = None
        self.gate_expired = False
        self.recover_until = None
        self._moving = False

        self.cmd_pub = rospy.Publisher("/navigation/cmd_vel", Twist, queue_size=10)
        self.path_pub = rospy.Publisher("/navigation/global_path", Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/navigation/status", String, queue_size=10)
        self.internal_pub = rospy.Publisher("/navigation/internal_state", String, queue_size=10)

        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
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
            return  # /map is re-published at 1 Hz; skip identical grids
        cm = OccupancyCostmap(inflation_cells=self.inflation_cells)
        cm.update_map(msg)
        self.costmap = cm  # atomic swap
        self._last_map_key = key
        rospy.loginfo("[navigation_node] map updated: %dx%d @ %.2f m", cm.width, cm.height, cm.resolution)

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
            self._set_status(PLANNING)  # published BEFORE the service call returns

    # --------------------------------------------------------------- status
    def _set_status(self, state):
        self.status = state
        self.status_pub.publish(String(data=state))

    def _set_status_if_changed(self, state):
        if self.status != state:
            self._set_status(state)

    def _finish(self, goal, state, why):
        with self.lock:
            if self.goal is not goal:      # a newer goal replaced this one meanwhile
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
        self._moving = any(abs(v) > 1e-6 for v in (vx, vy, wz))

    def _stop(self):
        if self._moving:
            self.cmd_pub.publish(Twist())
            self._moving = False

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
            ps.header.stamp = rospy.Time(0)  # latest available transform
            ps = self.tf_listener.transformPose(self.global_frame, ps)
        q = ps.pose.orientation
        if q.x == q.y == q.z == q.w == 0.0:
            yaw = None
        else:
            yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (ps.pose.position.x, ps.pose.position.y, yaw)

    def _publish_path(self, path):
        msg = Path()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = rospy.Time.now()
        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

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
            goal["path"] = None          # the map->odom transform may jump: re-plan afterwards
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
            self._set_status_if_changed(PLANNING)  # suspended, still an ACTIVE state
        self._publish_cmd(0.0, 0.0, self._sweep_wz(elapsed - self.still_time))

    def _leave_gate(self):
        self.gate_t0, self.gate_expired = None, False
        self._stop()

    # ---------------------------------------------------------- state machine
    def tick(self):
        now = rospy.Time.now().to_sec()
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
        pose, cm = self.get_robot_pose(), self.costmap
        target, why = None, ""
        if pose is not None and cm is not None and cm.grid is not None:
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
                pose = None  # goal frame not resolvable yet: treat like missing pose
        if pose is None or cm is None or cm.grid is None:
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
            self._recover(goal, now, "planner found no path")
            return
        path[-1] = (target[0], target[1])       # end exactly on the goal, not the cell centre
        goal["path"], goal["target"] = path, target
        goal["progress"] = (now, pose[0], pose[1], pose[2])
        self._publish_path(path)
        rospy.loginfo("[navigation_node] path with %d points to (%.2f, %.2f, yaw %s)", len(path),
                      target[0], target[1], "any" if target[2] is None else "%.2f" % target[2])
        self._set_status_if_changed(FOLLOWING)

    def _follow(self, goal, now):
        self.internal = "FOLLOWING"
        pose = self.get_robot_pose()
        if pose is None:
            self._stop()
            return
        cmd, reached = self.controller.compute_cmd_vel(pose, goal["path"], goal["target"][2])
        if reached:
            self._finish(goal, SUCCESS, "within %.2f m / %.2f rad" % (XY_TOL, YAW_TOL))
            return
        t_ref, x_ref, y_ref, yaw_ref = goal["progress"]
        if (math.hypot(pose[0] - x_ref, pose[1] - y_ref) > self.stuck_dist
                or abs(math.atan2(math.sin(pose[2] - yaw_ref), math.cos(pose[2] - yaw_ref))) > 0.05):
            goal["progress"] = (now, pose[0], pose[1], pose[2])
        elif now - t_ref > self.stuck_window:
            self._recover(goal, now, "no progress for %.0f s" % self.stuck_window)
            return
        self._set_status_if_changed(FOLLOWING)
        self.cmd_pub.publish(cmd)
        self._moving = True

    # ------------------------------------------------------------------ loop
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self.tick()
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
