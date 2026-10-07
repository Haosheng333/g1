#!/usr/bin/env python3
"""ROS1 node for Module 3: exposes /arm/move_arm_joints.

Starts the same background 250 Hz process as the original arm_control/DIP.py
smoke test (shared memory for target/measured, a pipe for commands), but as
a long-lived ROS node instead of a one-shot script, and exposes waypoint
execution as a service so other packages (the orchestrator, or arm_planning)
can drive the arm without touching the DDS layer or multiprocessing details
directly.

Untested on this machine -- no ROS, no unitree_sdk2py, no real robot here.
Verify on the robot before relying on it.
"""

from __future__ import annotations

import multiprocessing as mp
from multiprocessing import shared_memory

import numpy as np
import rospy

from arm_control.arm_controller import Execute_Waypoint_Path, Run_Arm_Controller, left_target, right_target
from arm_control.srv import MoveArmJoints, MoveArmJointsResponse

DEFAULT_TIMEOUT_S = 5.0


class ArmControlNode:
    def __init__(self) -> None:
        mp.set_start_method("spawn", force=True)

        self.shm = shared_memory.SharedMemory(create=True, size=10 * np.dtype(np.float64).itemsize)
        self.target = np.ndarray((10,), dtype=np.float64, buffer=self.shm.buf)
        self.target_lock = mp.Lock()

        self.measured_shm = shared_memory.SharedMemory(create=True, size=10 * np.dtype(np.float64).itemsize)
        self.measured = np.ndarray((10,), dtype=np.float64, buffer=self.measured_shm.buf)
        self.measured_lock = mp.Lock()

        with self.target_lock:
            self.target[0:5] = left_target
            self.target[5:10] = right_target

        self.parent_conn, child_conn = mp.Pipe()
        self.process = mp.Process(
            target=Run_Arm_Controller,
            args=(self.shm.name, self.target_lock, self.measured_shm.name, self.measured_lock, child_conn),
        )
        self.process.start()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self) -> None:
        # Ramp the arm_sdk weight to 0 (back to the whole-body stack) before
        # killing the process, instead of cutting it off at full weight.
        self.parent_conn.send("RELEASE")
        rospy.sleep(1.2)
        self.process.terminate()
        self.process.join()
        self.shm.close()
        self.shm.unlink()
        self.measured_shm.close()
        self.measured_shm.unlink()

    def handle_move_arm_joints(self, req) -> MoveArmJointsResponse:
        if req.arm not in ("left", "right"):
            return MoveArmJointsResponse(success=False, message=f"arm must be 'left' or 'right', got {req.arm!r}")
        if len(req.path_flat) == 0 or len(req.path_flat) % 5 != 0:
            return MoveArmJointsResponse(success=False, message="path_flat length must be a positive multiple of 5")

        path = np.asarray(req.path_flat, dtype=np.float64).reshape(-1, 5)
        timeout = req.timeout_per_waypoint if req.timeout_per_waypoint > 0 else DEFAULT_TIMEOUT_S

        ok = Execute_Waypoint_Path(
            self.target, self.target_lock, self.measured, self.measured_lock, req.arm, path, timeout_per_waypoint=timeout
        )
        message = "" if ok else "timed out before reaching a waypoint"
        return MoveArmJointsResponse(success=ok, message=message)


def main() -> None:
    rospy.init_node("arm_control_node")
    node = ArmControlNode()
    rospy.Service("/arm/move_arm_joints", MoveArmJoints, node.handle_move_arm_joints)
    rospy.spin()


if __name__ == "__main__":
    main()
