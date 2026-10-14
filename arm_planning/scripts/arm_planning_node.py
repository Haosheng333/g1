#!/usr/bin/env python3
"""ROS1 Noetic planning-service wrapper around the Module 2 arm planner.

Exposes the two planning interfaces of the specification (doc sections 5, 6
and 9.2, renamed into this repository's /arm/* namespace to sit beside
arm_control's /arm/move_arm_joints):

    /arm/plan_to_pose           arm_planning/PlanToPose
    /arm/plan_grasp_from_point  arm_planning/PlanGraspFromPoint

and a readiness pair the orchestrator preflights against, mirroring
g1_slam's /slam/manager/{state,status} pattern:

    /arm/planner/ready          std_msgs/Bool   (latched)
    /arm/planner/status         std_msgs/String (latched)

PLANNING ONLY (doc 2.1, 12.1). This node constructs no controller client, no
publisher to any command topic, and no service proxy to arm_control. A plan is
not authorization to move; execution is the orchestrator's separate, gated
call to /arm/move_arm_joints.

FRAMES. target_pose and object_xyz are torso_link coordinates (doc 2.3, 13.4).
This node performs NO frame conversion: the pelvis/base_link -> torso_link
transform is an unresolved integration decision (slam/config/g1_tf.yaml marks
pelvis -> base_link UNRESOLVED, and the two vendored URDFs disagree on where
torso_link sits by 10 mm), so converting here would be a guess.

STARTUP. torch, scipy, trimesh and fcl are imported lazily inside
_build_planner so that a missing dependency, a missing checkpoint or a missing
mesh produces a clear NOT_READY diagnostic and a node that still answers
services with success=false -- rather than a traceback at import time that
leaves the orchestrator unable to tell what is wrong. Set
~exit_on_init_failure:=true to make startup failures fatal instead.
"""
from __future__ import annotations

import os
import sys
import threading

import rospy
from std_msgs.msg import Bool, String

from arm_planning import plan_conversion as pc
from arm_planning.srv import (
    PlanGraspFromPoint,
    PlanGraspFromPointResponse,
    PlanToPose,
    PlanToPoseResponse,
)

#: Dependencies the planner chain needs, with the module that pulls each one in.
RUNTIME_DEPENDENCIES = (
    ("numpy", "every planner module"),
    ("yaml", "arm_kinematics.yaml loading"),
    ("torch", "guarded_ik_solver (learned IK checkpoints)"),
    ("scipy", "collision_aware_ik (least_squares fallback)"),
    ("trimesh", "g1_collision_probe (STL loading)"),
    ("fcl", "g1_collision_probe (collision queries)"),
)


def _package_root() -> str:
    """Package share directory via rospkg, never the current directory."""
    import rospkg

    return rospkg.RosPack().get_path("arm_planning")


def _missing_dependencies() -> list:
    import importlib

    missing = []
    for name, why in RUNTIME_DEPENDENCIES:
        try:
            importlib.import_module(name)
        except ImportError:
            missing.append("%s (needed by %s)" % (name, why))
    return missing


class ArmPlanningNode:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._planner = None          # Module2Planner
        self._grasp_planner = None    # Module2GraspPlanner
        self._blockers: list = []
        self._assets = None

        self.ready_pub = rospy.Publisher("/arm/planner/ready", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/arm/planner/status", String, queue_size=1, latch=True)
        self._publish_status(False, "initializing")

        self._project_root = rospy.get_param("~project_root", "") or _package_root()
        self._load()

    # ------------------------------------------------------------------ status
    def _publish_status(self, ready: bool, text: str) -> None:
        self.ready_pub.publish(Bool(data=bool(ready)))
        self.status_pub.publish(String(data=text))

    @property
    def ready(self) -> bool:
        return self._grasp_planner is not None

    def _not_ready_message(self) -> str:
        return "NOT_READY: " + "; ".join(self._blockers)

    # ------------------------------------------------------------------ startup
    def _load(self) -> None:
        """Resolve assets, check dependencies, build the planner once."""
        self._blockers = []

        self._assets = pc.resolve_assets(self._project_root)
        if not self._assets.ready:
            self._blockers.append(self._assets.blocker_message())

        missing = _missing_dependencies()
        if missing:
            self._blockers.append("missing python dependencies: " + ", ".join(missing))

        if self._blockers:
            rospy.logerr("[arm_planning] %s", self._not_ready_message())
            self._publish_status(False, self._not_ready_message())
            if rospy.get_param("~exit_on_init_failure", False):
                rospy.signal_shutdown("arm planner assets or dependencies unavailable")
            return

        try:
            self._build_planner()
        except Exception as exc:  # noqa: BLE001 - any planner import/build failure is a blocker
            self._blockers.append("planner construction failed: %s: %s" % (type(exc).__name__, exc))
            rospy.logerr("[arm_planning] %s", self._not_ready_message())
            self._publish_status(False, self._not_ready_message())
            if rospy.get_param("~exit_on_init_failure", False):
                rospy.signal_shutdown("arm planner construction failed")
            return

        text = "READY: planner loaded from %s (checkpoints in %s/)" % (
            self._project_root,
            self._assets.checkpoint_dir_used,
        )
        rospy.loginfo("[arm_planning] %s", text)
        self._publish_status(True, text)

    def _build_planner(self) -> None:
        """Import the flat planner modules and load models/masks ONCE."""
        scripts_dir = os.path.join(self._project_root, "scripts")
        if scripts_dir not in sys.path:
            # The planner modules import each other flat (doc 8.3 layout), so
            # their directory has to be importable. Appended, not prepended,
            # to avoid shadowing installed packages.
            sys.path.append(scripts_dir)

        from module2_grasp_planner import Module2GraspPlanner
        from module2_planner import Module2Planner

        assets = self._assets
        rospy.loginfo("[arm_planning] loading IK checkpoints and 2 x 16 MB masks (takes a few s)")
        self._planner = Module2Planner(
            config_path=assets.config_path,
            left_checkpoint=assets.checkpoints["left"],
            right_checkpoint=assets.checkpoints["right"],
            left_mask=assets.masks["left"],
            right_mask=assets.masks["right"],
            project_root=self._project_root,
            max_position_error_m=rospy.get_param("~max_position_error_m", 0.01),
            max_rx_error_rad=rospy.get_param("~max_rx_error_rad", 0.10),
            max_rz_error_rad=rospy.get_param("~max_rz_error_rad", 0.15),
            search_radius=rospy.get_param("~search_radius", 3),
        )
        self._grasp_planner = Module2GraspPlanner(point_planner=self._planner)

    # ------------------------------------------------------- /arm/plan_to_pose
    def handle_plan_to_pose(self, req) -> PlanToPoseResponse:
        reject = pc.validate_plan_to_pose_request(req.arm, req.current_joints, req.target_pose)
        if reject:
            return PlanToPoseResponse(success=False, message=reject)
        if not self.ready:
            return PlanToPoseResponse(success=False, message=self._not_ready_message())

        try:
            with self._lock:  # FCL collision objects are mutable; serialize requests
                result = self._planner.plan_to_pose(
                    arm=req.arm,
                    current_joints=list(req.current_joints),
                    target_pose=list(req.target_pose),
                )
        except Exception as exc:  # noqa: BLE001 - never leave the caller without a verdict
            rospy.logerr("[arm_planning] plan_to_pose raised: %s: %s", type(exc).__name__, exc)
            return PlanToPoseResponse(
                success=False, message="planner raised %s: %s" % (type(exc).__name__, exc)
            )

        return self._to_pose_response(result)

    def _to_pose_response(self, result) -> PlanToPoseResponse:
        response = PlanToPoseResponse(
            success=bool(result.success),
            message=str(result.message),
            joint_names=list(result.joint_names),
            position_error_m=pc.nan_if_none(result.position_error_m),
            rx_error_rad=pc.nan_if_none(result.rx_error_rad),
            rz_error_rad=pc.nan_if_none(result.rz_error_rad),
            nodes_expanded=int(result.nodes_expanded),
            exact_goal_cell_blocked=bool(result.exact_goal_cell_blocked),
        )
        if not result.success:
            # Hard-failure rule (doc 12.1): no partial path ever leaves this node.
            return response
        try:
            flat, count = pc.flatten_path(result.joint_path)
        except ValueError as exc:
            return PlanToPoseResponse(
                success=False, message="planner returned a malformed path: %s" % exc
            )
        if count == 0:
            return PlanToPoseResponse(
                success=False, message="planner reported success with an empty path"
            )
        response.flattened_joint_path = flat
        response.waypoint_count = count
        response.verified_goal_joints = [float(v) for v in result.verified_goal_joints]
        return response

    # ------------------------------------------------ /arm/plan_grasp_from_point
    def handle_plan_grasp_from_point(self, req) -> PlanGraspFromPointResponse:
        reject = pc.validate_plan_grasp_request(req.arm, req.current_joints, req.object_xyz)
        if reject:
            return PlanGraspFromPointResponse(success=False, message=reject)
        if not self.ready:
            return PlanGraspFromPointResponse(success=False, message=self._not_ready_message())

        try:
            with self._lock:
                result = self._grasp_planner.plan_grasp_from_point(
                    arm=req.arm,
                    current_joints=list(req.current_joints),
                    object_xyz=list(req.object_xyz),
                )
        except Exception as exc:  # noqa: BLE001
            rospy.logerr(
                "[arm_planning] plan_grasp_from_point raised: %s: %s", type(exc).__name__, exc
            )
            return PlanGraspFromPointResponse(
                success=False, message="planner raised %s: %s" % (type(exc).__name__, exc)
            )

        return self._to_grasp_response(result)

    def _to_grasp_response(self, result) -> PlanGraspFromPointResponse:
        response = PlanGraspFromPointResponse(
            success=bool(result.success),
            message=str(result.message),
            joint_names=list(result.joint_names),
            pregrasp_pose=[float(v) for v in result.pregrasp_pose],
            grasp_pose=[float(v) for v in result.grasp_pose],
            lift_pose=[float(v) for v in result.lift_pose],
            retract_pose=[float(v) for v in result.retract_pose],
        )
        if not result.success:
            return response
        try:
            names, counts, flat = pc.flatten_segments(result.segments)
        except (ValueError, KeyError, TypeError) as exc:
            return PlanGraspFromPointResponse(
                success=False, message="planner returned malformed segments: %s" % exc
            )
        response.segment_names = names
        response.segment_waypoint_counts = counts
        response.flattened_segment_paths = flat
        response.final_joints = [float(v) for v in result.final_joints]
        return response


def main() -> None:
    rospy.init_node("arm_planning_node")
    node = ArmPlanningNode()
    rospy.Service("/arm/plan_to_pose", PlanToPose, node.handle_plan_to_pose)
    rospy.Service("/arm/plan_grasp_from_point", PlanGraspFromPoint, node.handle_plan_grasp_from_point)
    if node.ready:
        rospy.loginfo("[arm_planning] planning services up (planning only, no execution)")
    else:
        rospy.logwarn(
            "[arm_planning] services up but NOT_READY; every request will be refused: %s",
            node._not_ready_message(),
        )
    rospy.spin()


if __name__ == "__main__":
    main()
