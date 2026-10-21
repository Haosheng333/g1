#!/usr/bin/env python3
"""Handover, the g1_intellect way, for DIP.

Takes the arm that holds the can to a handover pose in front of the robot,
arms g1_hands' tactile auto-release monitor, and when the monitor reports
that a human pulled the object: opens the hand, settles, homes that arm.
This is main_process's _run_handover / _run_release / run_auto_release
flow, without the rest of main_process.

Services (all std_srvs/Trigger):
    ~start             drive to the handover pose and arm the monitor
    ~run_auto_release  release -> settle -> GoHomeArm (the monitor calls this;
                       you can call it by hand as a manual release)
    ~abort             disarm the monitor, leave the arm where it is
Topic:
    ~state             std_msgs/String, latched: idle | moving | armed | releasing | done | error:<why>

Needs: /g1_arm_simple (arm_grasp), /g1_hand_<arm>/release (g1_hands on PC2),
/auto_release_monitor/{arm,disarm} (g1_hands auto_release_monitor_node, started
by handover.launch with run_auto_release_service pointing here).
"""

from __future__ import annotations

import math
import threading
import time

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from arm_grasp.srv import (
    GoHomeArm, GoHomeArmRequest, MoveArmJoints, MoveArmJointsRequest, PlanToPose, PlanToPoseRequest,
)
from g1_hands.srv import ArmAutoRelease, ArmAutoReleaseRequest, DisarmAutoRelease, DisarmAutoReleaseRequest

from handover.handover_flow import interp_joint_waypoints, unflatten


class HandoverNode:
    def __init__(self) -> None:
        self.arm = str(rospy.get_param("~arm", "left")).strip().lower()
        if self.arm not in ("left", "right"):
            raise SystemExit(f"~arm must be left or right, got {self.arm!r}")
        self.label = str(rospy.get_param("~label", "bottle"))
        self.target = [float(v) for v in rospy.get_param("~target_xyz", [0.35, 0.0, 0.15])]
        pre = [float(v) for v in rospy.get_param("~pre_astar_deg", [20.0, 25.0, 0.0, 49.8, 0.0])]
        if self.arm == "right":
            pre[1] = -pre[1]  # mirror the shoulder roll, as main_process does
        self.pre_astar_rad = [math.radians(v) for v in pre]
        self.hold_sec = float(rospy.get_param("~hold_sec", 0.5))
        self.release_settle_sec = float(rospy.get_param("~release_settle_sec", 1.5))
        self.max_step_deg = float(rospy.get_param("~max_step_deg", 10.0))
        self.motion_timeout = float(rospy.get_param("~motion_timeout", 8.0))
        self.service_wait = float(rospy.get_param("~service_wait_sec", 5.0))
        arm_node = rospy.get_param("~arm_node", "/g1_arm_simple")
        self.move_name = f"{arm_node}/move_arm_joints"
        self.plan_name = f"{arm_node}/plan_to_pose"
        self.home_name = f"{arm_node}/GoHomeArm"
        self.release_name = rospy.get_param("~release_service", f"/g1_hand_{self.arm}/release")
        self.monitor_arm_name = rospy.get_param("~monitor_arm_service", "/auto_release_monitor/arm")
        self.monitor_disarm_name = rospy.get_param("~monitor_disarm_service", "/auto_release_monitor/disarm")

        self._lock = threading.Lock()
        self._state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self._set_state("idle")
        rospy.Service("~start", Trigger, self._start_cb)
        rospy.Service("~run_auto_release", Trigger, self._release_cb)
        rospy.Service("~abort", Trigger, self._abort_cb)
        rospy.loginfo("[handover] arm=%s label=%s target=%s hold=%.1fs settle=%.1fs",
                      self.arm, self.label, self.target, self.hold_sec, self.release_settle_sec)

    # ------------------------------------------------------------------ helpers
    def _set_state(self, s: str) -> None:
        self.state = s
        self._state_pub.publish(String(data=s))
        rospy.loginfo("[handover] state: %s", s)

    def _call(self, name, srv_type, req):
        rospy.wait_for_service(name, timeout=self.service_wait)
        return rospy.ServiceProxy(name, srv_type)(req)

    def _move(self, joints, tag) -> tuple[bool, str]:
        resp = self._call(self.move_name, MoveArmJoints,
                          MoveArmJointsRequest(arm=self.arm, joint_angles=list(joints), wait=True,
                                               timeout=self.motion_timeout))
        if not resp.success:
            return False, f"{tag}: move_arm_joints failed: {resp.message}"
        return True, ""

    def _disarm(self) -> None:
        try:
            self._call(self.monitor_disarm_name, DisarmAutoRelease, DisarmAutoReleaseRequest(arm=self.arm))
        except Exception as exc:  # noqa: BLE001 -- disarm is best effort
            rospy.logwarn("[handover] disarm failed: %s", exc)

    # ------------------------------------------------------------------ services
    def _start_cb(self, _req) -> TriggerResponse:
        if not self._lock.acquire(blocking=False):
            return TriggerResponse(success=False, message="handover already in progress")
        try:
            return self._start()
        except Exception as exc:  # noqa: BLE001
            self._set_state(f"error:{type(exc).__name__}: {exc}")
            return TriggerResponse(success=False, message=f"{type(exc).__name__}: {exc}")
        finally:
            self._lock.release()

    def _start(self) -> TriggerResponse:
        self._set_state("moving")
        # 1) pre-A* "elbow up" start pose, straight joint move
        ok, msg = self._move(self.pre_astar_rad, "pre_astar")
        if not ok:
            self._set_state(f"error:{msg}")
            return TriggerResponse(success=False, message=msg)
        # 2) plan the handover pose (orientation free), A* from the current joints
        x, y, z = self.target
        plan = self._call(self.plan_name, PlanToPose,
                          PlanToPoseRequest(arm=self.arm, x=x, y=y, z=z, rx=float("nan"), rz=float("nan"),
                                            astar_override=PlanToPoseRequest.ASTAR_USE_DEFAULT))
        if not plan.success:
            self._set_state(f"error:plan_to_pose: {plan.message}")
            return TriggerResponse(success=False, message=f"plan_to_pose failed: {plan.message}")
        path = unflatten(plan.pre_path_flat, plan.pre_waypoint_count)
        if not path:
            if len(plan.target_joint_angles) == 5:
                path = [list(plan.target_joint_angles)]
            else:
                self._set_state("error:plan_to_pose returned no path")
                return TriggerResponse(success=False, message="plan_to_pose returned no path")
        # 3) drive it, interpolated so each move is short and the arm keeps moving
        last = list(path[0])
        smoothed = [last]
        for wp in path[1:]:
            smoothed.extend(interp_joint_waypoints(last, wp, self.max_step_deg))
            last = list(wp)
        rospy.loginfo("[handover] driving %d waypoints to (%.2f, %.2f, %.2f)", len(smoothed), x, y, z)
        for i, wp in enumerate(smoothed):
            ok, msg = self._move(wp, f"handover[{i + 1}/{len(smoothed)}]")
            if not ok:
                self._set_state(f"error:{msg}")
                return TriggerResponse(success=False, message=msg)
        # 4) settle, then arm the tactile monitor
        if self.hold_sec > 0:
            time.sleep(self.hold_sec)
        resp = self._call(self.monitor_arm_name, ArmAutoRelease, ArmAutoReleaseRequest(arm=self.arm, label=self.label))
        if not resp.success:
            self._set_state(f"error:monitor arm: {resp.message}")
            return TriggerResponse(success=False, message=f"at the handover pose but the monitor refused to arm: {resp.message}")
        self._set_state("armed")
        return TriggerResponse(success=True, message=f"at the handover pose, monitor armed on the {self.arm} hand; "
                                                     "pull the object to trigger the release")

    def _release_cb(self, _req) -> TriggerResponse:
        if not self._lock.acquire(blocking=False):
            return TriggerResponse(success=False, message="handover busy")
        try:
            self._set_state("releasing")
            self._disarm()
            resp = self._call(self.release_name, Trigger, TriggerRequest())
            if not resp.success:
                self._set_state(f"error:release: {resp.message}")
                return TriggerResponse(success=False, message=f"hand release failed: {resp.message}")
            if self.release_settle_sec > 0:
                time.sleep(self.release_settle_sec)
            home = self._call(self.home_name, GoHomeArm,
                              GoHomeArmRequest(arm=self.arm, astar_override=GoHomeArmRequest.ASTAR_USE_DEFAULT))
            if not home.success:
                self._set_state(f"error:GoHomeArm: {home.message}")
                return TriggerResponse(success=False, message=f"hand opened, but GoHomeArm failed: {home.message}")
            self._set_state("done")
            return TriggerResponse(success=True, message=f"released and {self.arm} arm homed")
        except Exception as exc:  # noqa: BLE001
            self._set_state(f"error:{type(exc).__name__}: {exc}")
            return TriggerResponse(success=False, message=f"{type(exc).__name__}: {exc}")
        finally:
            self._lock.release()

    def _abort_cb(self, _req) -> TriggerResponse:
        self._disarm()
        self._set_state("idle")
        return TriggerResponse(success=True, message="monitor disarmed; arm left where it is")


def main() -> None:
    rospy.init_node("handover")
    HandoverNode()
    rospy.spin()


if __name__ == "__main__":
    main()
