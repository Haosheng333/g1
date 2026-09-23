import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize
)

from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_
)

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_,
    LowState_
)

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import multiprocessing as mp
from multiprocessing import shared_memory

left_target = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float64)
right_target = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float64)

class JointIndex:
    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19

    #right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26

class Mode:
    PR = 0
    AB = 1

# 250Hz control loop

def Control_Loop_250Hz(controller):
    next_deadline = time.perf_counter()
    while True:
        controller.Update_Arm_Command()
        controller.Publish_Arm_Command()

        next_deadline += controller.control_dt
        sleep_time = next_deadline - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_deadline = time.perf_counter()

def Run_Arm_Controller(shm_name, target_lock, measured_shm_name, measured_lock, child_conn):
    ChannelFactoryInitialize(0)

    shm = shared_memory.SharedMemory(name = shm_name)
    target = np.ndarray((10,), dtype = np.float64, buffer = shm.buf)

    measured_shm = shared_memory.SharedMemory(name = measured_shm_name)
    measured = np.ndarray((10,), dtype = np.float64, buffer = measured_shm.buf)

    controller = ArmController(target, target_lock, measured, measured_lock, child_conn)
    Control_Loop_250Hz(controller)

    shm.close()
    measured_shm.close()

# Arm controller

class ArmController:
    def __init__(self, target, target_lock, measured, measured_lock, child_conn):

        self.target = target
        self.target_lock = target_lock
        self.measured = measured
        self.measured_lock = measured_lock
        self.child_conn = child_conn
        
        #Arm target position
        self.left_target = None
        self.right_target = None

        #Arm conmmand position
        self.left_command = None
        self.right_command = None

        #Publisher
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)  
        self.arm_sdk_publisher.Init()

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  # Command message to be pulished
        self.crc = CRC()

        #Subscriber
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)  # Subscribe to lowstate DDS, LowState_ defined expected msg type
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)  # Initialise subscriber, call LowStateHandler when there is new lowstate msg, queue depth 10

        self.low_state = None  # Variable to store latest lowstate msg

        #Joint velocity limit
        self.control_dt = 1.0/250.0  # Control period 250Hz
        self.max_joint_velocity = np.deg2rad(20.0)  # Max joint velocity: 20 deg/s
        self.max_joint_step = self.max_joint_velocity * self.control_dt  # Max joint position change per cycle

        #Arm sdk weight (0 = released to whole-body stack, 1 = full arm control)
        self.weight = 1.0
        self.releasing = False
        release_seconds = 1.0
        self.release_step = 1.0 / (release_seconds / self.control_dt)

    def Poll_Control_Pipe(self):
        while self.child_conn.poll():
            msg = self.child_conn.recv()
            if msg == "RELEASE":
                self.releasing = True

    def Publish_Arm_Command(self):
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)
        
    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

    # Get arm position
    def Get_Left_Arm_Position(self):
        if self.low_state is None:
            return None
        return np.array([
            self.low_state.motor_state[JointIndex.LeftShoulderPitch].q,
            self.low_state.motor_state[JointIndex.LeftShoulderRoll].q,
            self.low_state.motor_state[JointIndex.LeftShoulderYaw].q,
            self.low_state.motor_state[JointIndex.LeftElbow].q,
            self.low_state.motor_state[JointIndex.LeftWristRoll].q
            ])

    def Get_Right_Arm_Position(self):
        if self.low_state is None:
            return None
        return np.array([
            self.low_state.motor_state[JointIndex.RightShoulderPitch].q,
            self.low_state.motor_state[JointIndex.RightShoulderRoll].q,
            self.low_state.motor_state[JointIndex.RightShoulderYaw].q,
            self.low_state.motor_state[JointIndex.RightElbow].q,
            self.low_state.motor_state[JointIndex.RightWristRoll].q
            ])

    #q - target position, dq target velocity, tau - feedforward torque, kp - position gain, kd - velocity gain
    def Update_Arm_Command(self):
        if self.low_state is None:
            return

        left_measured = self.Get_Left_Arm_Position()
        right_measured = self.Get_Right_Arm_Position()
        with self.measured_lock:
            self.measured[0:5] = left_measured
            self.measured[5:10] = right_measured

        self.Poll_Control_Pipe()

        # Ramp weight 1 -> 0 over ~1 second once a release has been requested.
        if self.releasing:
            self.weight = max(0.0, self.weight - self.release_step)

        with self.target_lock:
            self.left_target = self.target[0:5].copy()
            self.right_target = self.target[5:10].copy()

        if self.left_command is None:
            self.left_command = left_measured.copy()
        
        if self.left_target is not None:
            error = self.left_target - self.left_command
            step = np.clip(error, -self.max_joint_step, self.max_joint_step)
            self.left_command += step 

        if self.right_command is None:
            self.right_command = right_measured.copy()

        if self.right_target is not None:
            error = self.right_target - self.right_command
            step = np.clip(error, -self.max_joint_step, self.max_joint_step)
            self.right_command += step
        
        self.low_cmd.mode_pr = Mode.PR
        self.low_cmd.mode_machine = self.low_state.mode_machine
        
        left_joints = [
            JointIndex.LeftShoulderPitch,
            JointIndex.LeftShoulderRoll,
            JointIndex.LeftShoulderYaw,
            JointIndex.LeftElbow,
            JointIndex.LeftWristRoll
            ]

        for i in range(5):
            joint = left_joints[i]
            self.low_cmd.motor_cmd[joint].q = self.left_command[i]
            self.low_cmd.motor_cmd[joint].mode = 1
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].kp = 40.0
            self.low_cmd.motor_cmd[joint].kd = 1.0

        right_joints = [
            JointIndex.RightShoulderPitch,
            JointIndex.RightShoulderRoll,
            JointIndex.RightShoulderYaw,
            JointIndex.RightElbow,
            JointIndex.RightWristRoll]

        for i in range(5):
            joint = right_joints[i]
            self.low_cmd.motor_cmd[joint].q = self.right_command[i]
            self.low_cmd.motor_cmd[joint].mode = 1
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].kp = 40.0
            self.low_cmd.motor_cmd[joint].kd = 1.0

        self.low_cmd.motor_cmd[29].q = self.weight  # arm_sdk motion-mode weight

def Set_Target(target, target_lock, arm, q):
    """Called from the parent process to push a new 5-DOF target for
    'left' or 'right' into shared memory. Non-blocking except for the
    brief lock hold."""
    sl = slice(0, 5) if arm == "left" else slice(5, 10)
    with target_lock:
        target[sl] = np.asarray(q, dtype = np.float64)

ARRIVAL_TOL_RAD = 0.07
POLL_HZ = 50
POLL_DT = 1.0 / POLL_HZ

def Get_Measured(measured, measured_lock, arm):
    sl = slice(0, 5) if arm == "left" else slice(5, 10)
    with measured_lock:
        return measured[sl].copy()

def Wait_Until_Arrived(measured, measured_lock, arm, q_target, timeout = 5.0):
    """Poll the child's measured-position array at 50 Hz until every joint
    is within ARRIVAL_TOL_RAD of q_target, or timeout. Returns True on
    arrival, False on timeout."""
    deadline = time.perf_counter() + timeout
    q_target = np.asarray(q_target, dtype = np.float64)
    while time.perf_counter() < deadline:
        current = Get_Measured(measured, measured_lock, arm)
        if np.max(np.abs(current - q_target)) < ARRIVAL_TOL_RAD:
            return True
        time.sleep(POLL_DT)
    return False

def Execute_Waypoint_Path(target, target_lock, measured, measured_lock,
                           arm, path_rad, timeout_per_waypoint = 5.0):
    """Drive through a sequence of waypoints, blocking (from the caller's
    perspective) until each is reached before sending the next. Note:
    Set_Target itself never blocks — this function does, by design, since
    that's what a waypoint path needs."""
    for q in path_rad:
        Set_Target(target, target_lock, arm, q)
        if not Wait_Until_Arrived(measured, measured_lock, arm, q, timeout = timeout_per_waypoint):
            return False  # timed out — caller decides how to handle a stall
    return True

# Multiple processing

if __name__ == "__main__":
    mp.set_start_method("spawn")

    shm = shared_memory.SharedMemory(create = True, size = 10 * np.dtype(np.float64).itemsize)
    target = np.ndarray((10,), dtype = np.float64, buffer = shm.buf)
    target_lock = mp.Lock()

    measured_shm = shared_memory.SharedMemory(create = True, size = 10 * np.dtype(np.float64).itemsize)
    measured = np.ndarray((10,), dtype = np.float64, buffer = measured_shm.buf)
    measured_lock = mp.Lock()

    with target_lock:
        target[0:5] = left_target
        target[5:10] = right_target

    parent_conn, child_conn = mp.Pipe()

    arm_process = mp.Process(
        target = Run_Arm_Controller,
        args = (shm.name, target_lock, measured_shm.name, measured_lock, child_conn)
    )

    arm_process.start()

    parent_conn.send("NEW_TARGET")

    arm_process.join()

    shm.close()
    shm.unlink()
    measured_shm.close()
    measured_shm.unlink()
