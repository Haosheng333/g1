#!/usr/bin/env python3
"""Hardware-facing TEST DOUBLES for the pipeline integration test.

WHAT THIS IS, PRECISELY
Every service and topic here uses the **real generated** message/service
definitions (`g1_navigation/NavigateToObject`, `perception/GetObjectPosition`,
`std_srvs/Trigger`, `std_srvs/SetBool`, `sensor_msgs/JointState`,
`std_msgs/*`). What is faked is only the *behaviour behind* them: no
locomotion, no arm SDK, no hand SDK, no camera, no LiDAR. Nothing in this file
imports `unitree_sdk2py`, `bc_stark_sdk`, `ultralytics` or `torch`, so it cannot
reach a robot even if one were on the network.

It is deliberately NOT part of any launch file in the repository. It lives under
orchestrator/test/ and is started only by run_integration.sh, which also forces
an isolated ROS master.

SCENARIOS
The behaviour is switched at runtime by publishing a scenario name on
/test/scenario, so one process can serve every success and failure case without
restarting. Each scenario is named in the report.

FIXTURE VALUES are tagged in the log with FIXTURE= so they are traceable from
the transcript to this file. They are invented numbers for a test, not measured
properties of any robot.
"""
import math
import sys
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

from arm_control.srv import MoveArmJoints, MoveArmJointsResponse
from arm_planning.srv import (
    PlanGraspFromPoint,
    PlanGraspFromPointResponse,
    PlanToPose,
    PlanToPoseResponse,
)
from g1_navigation.srv import NavigateToObject, NavigateToObjectResponse
from perception.srv import GetObjectPosition, GetObjectPositionResponse

# --- deterministic fixtures -----------------------------------------------------------
#
# Perception answer. metres, in the frame perception actually hardcodes
# (m1_perception.py: frame_id="base_link"). The orchestrator stores it verbatim
# and performs NO conversion, which is the point of boundary 3.
FIXTURE_DETECTION = {"x": 0.412, "y": -0.133, "z": 0.907, "frame_id": "base_link"}

#: Joint state the slam TF publisher needs (slam/config/g1_tf.yaml).
FIXTURE_WAIST_JOINT = "waist_yaw_joint"
FIXTURE_WAIST_RAD = 0.0

#: The five arm joints the planner needs as `current_joints`, in the planner's
#: order. Radians. Invented test values; no real arm was measured.
FIXTURE_ARM_JOINTS = {
    "left":  [0.10, 0.20, 0.30, -0.40, 0.50],
    "right": [-0.10, -0.20, -0.30, -0.40, -0.50],
}
ARM_JOINT_ORDER = ("shoulder_pitch_joint", "shoulder_roll_joint",
                   "shoulder_yaw_joint", "elbow_joint", "wrist_roll_joint")

#: PLANNER DOUBLE OUTPUT. Four grasp segments, waypoint counts chosen so each
#: block is distinguishable, values a simple ramp so a mis-sliced path is
#: obvious. This is NOT the real planner: no IK, no collision check, no
#: workspace validation. A pass here says the ORCHESTRATOR WIRING is right, and
#: nothing whatever about arm_planning's algorithms.
FIXTURE_SEGMENTS = (("to_pregrasp", 3), ("approach", 2), ("lift", 2), ("retract", 4))
#: Waypoint count the pose planner double returns for plan_to_pose.
FIXTURE_POSE_WAYPOINTS = 5

# --- TF, published so the REAL /arm/convert_observation can be exercised -------------
# These are the slam (mode_10) numbers, because the live TF tree really is
# published from slam/urdf/g1_23dof_mode_10.urdf. The 10 mm convention offset to
# the planner's frame is arm_planning's job, not this file's.
TF_WAIST_ORIGIN = (-0.0039635, 0.0, 0.044)
#: Waist yaw the published TF carries. Settable at runtime on /test/waist_yaw.
FIXTURE_WAIST_YAW_RAD = 0.0

SCENARIOS = (
    "happy",                 # everything succeeds
    "nav_failure",           # navigation reports FAILURE after becoming ACTIVE
    "nav_stale_success",     # a leftover SUCCESS only, never ACTIVE -> saw_active must hold
    "nav_timeout",           # accepted, then silence: no terminal status at all
    "nav_reject",            # the navigate service itself returns success=false
    "perception_failure",    # detector found nothing
    "perception_nonfinite",  # NaN coordinates on a "successful" response
    "hand_failure",          # every hand preset returns success=false
    "release_failure",       # only /hand_*/release fails
    "no_joint_states",       # stop publishing /joint_states
    "no_arm_joints",         # /joint_states keeps the waist but drops the 5 arm joints
    "no_force",              # stop publishing force frames
    "planner_failure",       # the planner reports success=false with a reason
    "planner_malformed",     # success=true but the path length contradicts the count
    "planner_wrong_joint_order",   # success=true with a permuted joint_names
    "arm_exec_failure",      # /arm/move_arm_joints reports success=false
    "no_tf",                 # stop publishing TF: the conversion must refuse
)


class Doubles(object):
    def __init__(self):
        rospy.init_node("pipeline_doubles")
        self.scenario = str(rospy.get_param("~scenario", "happy"))
        self.side = str(rospy.get_param("~side", "right"))
        self.lock = threading.Lock()
        self.calls = []                      # ordered record, published for the test

        # --- latched readiness flags the orchestrator's preflight reads --------
        self.pub_slam = rospy.Publisher("/slam/manager/state", Bool, queue_size=1, latch=True)
        self.pub_planner = rospy.Publisher("/arm/planner/ready", Bool, queue_size=1, latch=True)
        self.pub_map = rospy.Publisher("/map", String, queue_size=1, latch=True)
        self.pub_nav_status = rospy.Publisher("/navigation/status", String, queue_size=1,
                                              latch=True)
        self.pub_joints = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.pub_force = rospy.Publisher("/test/force", Float32MultiArray, queue_size=1)
        self.pub_calls = rospy.Publisher("/test/calls", String, queue_size=50)
        self.pub_ready = rospy.Publisher("/test/doubles_ready", Bool, queue_size=1, latch=True)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._waist_yaw = FIXTURE_WAIST_YAW_RAD
        self.pub_tf_enabled = True
        rospy.Subscriber("/test/waist_yaw", Float32MultiArray, self._on_waist, queue_size=5)
        rospy.Subscriber("/test/scenario", String, self._on_scenario, queue_size=5)
        rospy.Subscriber("/test/force_cmd", Float32MultiArray, self._on_force_cmd, queue_size=5)
        rospy.Subscriber("/navigation/cancel", rospy.AnyMsg, self._on_cancel, queue_size=5)

        # --- hardware-facing services, real types, faked behaviour -------------
        rospy.Service("/navigation/navigate", NavigateToObject, self.navigate)
        rospy.Service("/navigation/enable_motion", SetBool, self.enable_motion)
        rospy.Service("/perception/get_object_position", GetObjectPosition, self.detect)
        for preset in ("pre_grasp", "grasp", "release"):
            rospy.Service("/hand_%s/%s" % (self.side, preset), Trigger,
                          self._hand_handler(preset))
        # arm_planning / arm_control DOUBLES, real generated types. The
        # orchestrator now calls these for real (plan_grasp, execute_segment,
        # the pose steps), so they return deterministic fixture paths.
        rospy.Service("/arm/plan_to_pose", PlanToPose, self.plan_to_pose)
        rospy.Service("/arm/plan_grasp_from_point", PlanGraspFromPoint,
                      self.plan_grasp_from_point)
        rospy.Service("/arm/move_arm_joints", MoveArmJoints, self.move_arm_joints)

        self.pub_slam.publish(Bool(data=True))
        self.pub_planner.publish(Bool(data=True))
        self.pub_map.publish(String(data="FIXTURE map present"))
        self.pub_force_enabled = True
        self.pub_joints_enabled = True
        self._force_values = [0.0, 0.0, 0.0]
        rospy.loginfo("[doubles] up; scenario=%s side=%s", self.scenario, self.side)
        self.pub_ready.publish(Bool(data=True))

    # ------------------------------------------------------------------ plumbing
    def _record(self, what, detail=""):
        with self.lock:
            entry = "%s|%s" % (what, detail)
            self.calls.append(entry)
        self.pub_calls.publish(String(data=entry))
        rospy.loginfo("[doubles] CALL %s", entry)

    def _on_scenario(self, msg):
        with self.lock:
            self.scenario = msg.data
            self.calls = []
        self.pub_joints_enabled = msg.data != "no_joint_states"
        self.pub_force_enabled = msg.data != "no_force"
        self.pub_tf_enabled = msg.data != "no_tf"
        # A fresh scenario starts from a clean status topic. "nav_stale_success"
        # deliberately leaves a SUCCESS behind instead.
        if msg.data == "nav_stale_success":
            self.pub_nav_status.publish(String(data="SUCCESS"))
        else:
            self.pub_nav_status.publish(String(data="IDLE"))
        rospy.loginfo("[doubles] scenario -> %s", msg.data)

    def _on_waist(self, msg):
        self._waist_yaw = float(msg.data[0]) if msg.data else 0.0
        rospy.loginfo("[doubles] TF waist yaw -> %.4f rad", self._waist_yaw)

    def _publish_tf(self):
        """pelvis -> torso_link (waist) and pelvis -> base_link (identity).

        Exactly the shape of the real tree: g1_slam publishes the waist joint
        from the mode_10 URDF, and system_bringup publishes base_link as an
        identity child of pelvis. tf2 composes torso_link <- base_link from them.
        """
        now = rospy.Time.now()
        out = []

        root = TransformStamped()
        root.header.stamp, root.header.frame_id = now, "world"
        root.child_frame_id = "pelvis"
        root.transform.rotation.w = 1.0
        out.append(root)

        torso = TransformStamped()
        torso.header.stamp, torso.header.frame_id = now, "pelvis"
        torso.child_frame_id = "torso_link"
        torso.transform.translation.x = TF_WAIST_ORIGIN[0]
        torso.transform.translation.y = TF_WAIST_ORIGIN[1]
        torso.transform.translation.z = TF_WAIST_ORIGIN[2]
        torso.transform.rotation.z = math.sin(self._waist_yaw / 2.0)
        torso.transform.rotation.w = math.cos(self._waist_yaw / 2.0)
        out.append(torso)

        base = TransformStamped()
        base.header.stamp, base.header.frame_id = now, "pelvis"
        base.child_frame_id = "base_link"
        base.transform.rotation.w = 1.0
        out.append(base)

        self._tf_broadcaster.sendTransform(out)

    def _on_force_cmd(self, msg):
        self._force_values = list(msg.data)
        rospy.loginfo("[doubles] force fixture -> %s", self._force_values)

    def _on_cancel(self, _msg):
        self._record("navigation/cancel")

    # ------------------------------------------------------------------ services
    def navigate(self, req):
        self._record("navigate", "class_name=%s approach=%.3f" % (
            req.class_name, req.approach_distance))
        if self.scenario == "nav_reject":
            return NavigateToObjectResponse(
                success=False, message="FIXTURE: goal rejected (unknown class)")
        threading.Thread(target=self._drive, daemon=True).start()
        return NavigateToObjectResponse(success=True, message="FIXTURE: goal accepted")

    def _drive(self):
        """Emit the status sequence the saw_active guard consumes."""
        scenario = self.scenario
        if scenario == "nav_timeout":
            rospy.loginfo("[doubles] nav_timeout: publishing no terminal status")
            return
        if scenario == "nav_stale_success":
            # Never becomes ACTIVE. The leftover SUCCESS is already latched.
            rospy.loginfo("[doubles] nav_stale_success: never going ACTIVE")
            return
        for state in ("PLANNING", "FOLLOWING"):
            rospy.sleep(0.15)
            self.pub_nav_status.publish(String(data=state))
        rospy.sleep(0.15)
        self.pub_nav_status.publish(
            String(data="FAILURE" if scenario == "nav_failure" else "SUCCESS"))

    def enable_motion(self, req):
        self._record("enable_motion", "data=%s" % req.data)
        return SetBoolResponse(success=True, message="FIXTURE: gate %s" % req.data)

    def detect(self, req):
        self._record("detect_object", "label=%s index=%d" % (req.label, req.index))
        if self.scenario == "perception_failure":
            return GetObjectPositionResponse(
                success=False, message="FIXTURE: no %s detected" % req.label)
        if self.scenario == "perception_nonfinite":
            nan = float("nan")
            return GetObjectPositionResponse(
                success=True, x=nan, y=nan, z=nan,
                frame_id=FIXTURE_DETECTION["frame_id"],
                message="FIXTURE: success with non-finite coordinates")
        d = FIXTURE_DETECTION
        return GetObjectPositionResponse(
            success=True, x=d["x"], y=d["y"], z=d["z"], frame_id=d["frame_id"],
            message="FIXTURE detection")

    # --- arm_planning / arm_control doubles --------------------------------
    def _joint_names(self, arm):
        return ["%s_%s" % (arm, j) for j in ARM_JOINT_ORDER]

    def _ramp(self, count, base):
        """count x 5 flattened joints: row i = base + i*0.01 per column."""
        out = []
        for i in range(count):
            out += [base + i * 0.01 + c * 0.001 for c in range(5)]
        return out

    def plan_grasp_from_point(self, req):
        self._record("plan_grasp_from_point",
                     "arm=%s object_xyz=%s n_current=%d" % (
                         req.arm, [round(v, 4) for v in req.object_xyz],
                         len(req.current_joints)))
        if self.scenario == "planner_failure":
            return PlanGraspFromPointResponse(
                success=False,
                message="FIXTURE: segment 'approach' has no collision-free path")
        names = [n for n, _ in FIXTURE_SEGMENTS]
        counts = [c for _, c in FIXTURE_SEGMENTS]
        flat = []
        for i, (_, c) in enumerate(FIXTURE_SEGMENTS):
            flat += self._ramp(c, 0.1 * (i + 1))
        joint_names = self._joint_names(req.arm)
        if self.scenario == "planner_wrong_joint_order":
            joint_names = list(reversed(joint_names))
        if self.scenario == "planner_malformed":
            flat = flat[:-5]        # one waypoint short of the declared counts
        return PlanGraspFromPointResponse(
            success=True, message="FIXTURE: four segments planned",
            joint_names=joint_names, segment_names=names,
            segment_waypoint_counts=counts, flattened_segment_paths=flat,
            final_joints=flat[-5:] if flat else [],
            pregrasp_pose=[0.18, -0.10, 0.15, 1.0, 0.0],
            grasp_pose=[0.30, -0.20, 0.00, 1.0, 0.0],
            lift_pose=[0.30, -0.20, 0.10, 1.0, 0.0],
            retract_pose=[0.15, -0.10, 0.10, 1.0, 0.0])

    def plan_to_pose(self, req):
        self._record("plan_to_pose", "arm=%s target_pose=%s n_current=%d" % (
            req.arm, [round(v, 4) for v in req.target_pose], len(req.current_joints)))
        if self.scenario == "planner_failure":
            return PlanToPoseResponse(
                success=False, message="FIXTURE: target pose is outside the workspace")
        count = FIXTURE_POSE_WAYPOINTS
        flat = self._ramp(count, 0.5)
        joint_names = self._joint_names(req.arm)
        if self.scenario == "planner_wrong_joint_order":
            joint_names = list(reversed(joint_names))
        if self.scenario == "planner_malformed":
            flat = flat[:-5]
        return PlanToPoseResponse(
            success=True, message="FIXTURE: pose planned",
            joint_names=joint_names, waypoint_count=count,
            flattened_joint_path=flat, verified_goal_joints=flat[-5:],
            position_error_m=0.0031, rx_error_rad=0.002, rz_error_rad=0.001,
            nodes_expanded=1234, exact_goal_cell_blocked=False)

    def move_arm_joints(self, req):
        n = len(req.path_flat) // 5
        self._record("move_arm_joints", "arm=%s waypoints=%d first=%s" % (
            req.arm, n,
            [round(v, 4) for v in req.path_flat[:5]] if req.path_flat else []))
        if self.scenario == "arm_exec_failure":
            return MoveArmJointsResponse(
                success=False, message="FIXTURE: waypoint 2 timed out")
        return MoveArmJointsResponse(
            success=True, message="FIXTURE: %d waypoints executed" % n)

    def _hand_handler(self, preset):
        def handler(_req):
            self._record("hand_%s" % preset)
            if self.scenario == "hand_failure":
                return TriggerResponse(success=False,
                                       message="FIXTURE: hand %s failed" % preset)
            if self.scenario == "release_failure" and preset == "release":
                return TriggerResponse(success=False,
                                       message="FIXTURE: release failed")
            # NOTE the asymmetry, and it is deliberate: this mirrors the real
            # hand contract. `release` returns success with NO settling
            # evidence, so the orchestrator must treat it as a command
            # acknowledgement only.
            return TriggerResponse(success=True,
                                   message="FIXTURE: %s commanded" % preset)
        return handler

    # ------------------------------------------------------------------ main loop
    def spin(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            if self.pub_joints_enabled:
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name = [FIXTURE_WAIST_JOINT]
                js.position = [FIXTURE_WAIST_RAD]
                if self.scenario != "no_arm_joints":
                    for side in ("left", "right"):
                        js.name += ["%s_%s" % (side, j) for j in ARM_JOINT_ORDER]
                        js.position += FIXTURE_ARM_JOINTS[side]
                self.pub_joints.publish(js)
            if self.pub_force_enabled:
                self.pub_force.publish(Float32MultiArray(data=self._force_values))
            if self.pub_tf_enabled:
                self._publish_tf()
            rate.sleep()


if __name__ == "__main__":
    try:
        Doubles().spin()
    except rospy.ROSInterruptException:
        sys.exit(0)
