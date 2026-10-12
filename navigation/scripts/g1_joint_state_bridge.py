#!/usr/bin/env python3
"""g1_joint_state_bridge - the REAL /joint_states that the TF chain needs.

Why this matters: g1_slam's g1_tf_publisher computes  camera_init -> pelvis  from FAST-LIO2 odometry
AND the waist yaw (the Mid-360 sits on torso_link; waist_yaw_joint is revolute).  Without the real
waist angle `pelvis` is rotated by the waist angle: navigation heading, relocalization seeds and
every goal yaw are wrong by that much.  The old bridge published all-zero positions with the log line
"Live joint state bridge active", which hid exactly this.  This one never fabricates data silently.

Modes (~mode)
  relay   (default) republish another JointState topic (~input_topic, REQUIRED) on /joint_states.
          Use this when the robot-control package already publishes joint states under another name.
  dds     read the Unitree G1 low-level state  rt/lowstate  with unitree_sdk2py (the same DDS channel the
          arm package uses, doc 0.3) and publish /joint_states.  Needs unitree_sdk2py importable by the
          python that runs this node and ~network_interface (e.g. enp2s0).  VERIFY the waist index
          on the robot before trusting it: see FIX_GUIDE_CD.md, step D1.
  fake    publish ZEROS.  Bench use only: refused unless ~allow_fake_zeros:=true, and it warns loudly
          every 5 s.  Never use it while the robot walks or while you test heading.

Common params: ~required_joint (waist_yaw_joint)  ~rate (50)  ~max_age (relay: 1.0 s, stale input is dropped)
"""
import sys
import threading

import rospy
from sensor_msgs.msg import JointState

# Unitree G1 motor order in rt/lowstate (unitree_sdk2py G1JointIndex).  The 23-DoF variant keeps these slots.
G1_MOTOR_INDEX = {
    "left_hip_pitch_joint": 0, "left_hip_roll_joint": 1, "left_hip_yaw_joint": 2,
    "left_knee_joint": 3, "left_ankle_pitch_joint": 4, "left_ankle_roll_joint": 5,
    "right_hip_pitch_joint": 6, "right_hip_roll_joint": 7, "right_hip_yaw_joint": 8,
    "right_knee_joint": 9, "right_ankle_pitch_joint": 10, "right_ankle_roll_joint": 11,
    "waist_yaw_joint": 12,
    "left_shoulder_pitch_joint": 15, "left_shoulder_roll_joint": 16, "left_shoulder_yaw_joint": 17,
    "left_elbow_joint": 18, "left_wrist_roll_joint": 19,
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23, "right_shoulder_yaw_joint": 24,
    "right_elbow_joint": 25, "right_wrist_roll_joint": 26,
}


def run_relay(pub, required, rate_hz, max_age):
    state = {"msg": None, "t": 0.0}
    lock = threading.Lock()
    topic = rospy.get_param("~input_topic", "")
    if not topic:
        rospy.logfatal("[joint_bridge] mode=relay needs ~input_topic (the topic that carries the real "
                       "sensor_msgs/JointState). Refusing to start.")
        sys.exit(2)

    def cb(msg):
        with lock:
            state["msg"], state["t"] = msg, rospy.Time.now().to_sec()

    rospy.Subscriber(topic, JointState, cb, queue_size=5)
    rospy.loginfo("[joint_bridge] relaying %s -> /joint_states", topic)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        with lock:
            msg, t = state["msg"], state["t"]
        if msg is None or rospy.Time.now().to_sec() - t > max_age:
            rospy.logwarn_throttle(5.0, "[joint_bridge] no fresh message on %s: publishing NOTHING "
                                   "(TF camera_init->pelvis will stop, which is the safe behaviour)", topic)
        elif required not in msg.name:
            rospy.logerr_throttle(5.0, "[joint_bridge] %s carries no '%s': publishing NOTHING. Names: %s",
                                  topic, required, list(msg.name)[:8])
        else:
            out = JointState()
            out.header.stamp = rospy.Time.now()
            out.name, out.position = list(msg.name), list(msg.position)
            out.velocity, out.effort = list(msg.velocity), list(msg.effort)
            pub.publish(out)
        rate.sleep()


def run_dds(pub, required, rate_hz):
    iface = rospy.get_param("~network_interface", "")
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError as exc:
        rospy.logfatal("[joint_bridge] mode=dds needs unitree_sdk2py in THIS python (%s). Use mode=relay "
                       "or install the SDK in the environment that runs ROS.", exc)
        sys.exit(2)
    if iface:
        ChannelFactoryInitialize(0, iface)
    else:
        ChannelFactoryInitialize(0)
    holder = {"low": None, "t": 0.0}

    def handler(msg):
        holder["low"], holder["t"] = msg, rospy.Time.now().to_sec()

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(handler, 10)
    names = list(G1_MOTOR_INDEX)
    if required not in G1_MOTOR_INDEX:
        rospy.logfatal("[joint_bridge] '%s' is not in the motor table", required)
        sys.exit(2)
    rospy.loginfo("[joint_bridge] reading rt/lowstate (iface=%s)", iface or "default")
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        low = holder["low"]
        if low is None or rospy.Time.now().to_sec() - holder["t"] > 1.0:
            rospy.logwarn_throttle(5.0, "[joint_bridge] no fresh rt/lowstate: publishing NOTHING")
        else:
            out = JointState()
            out.header.stamp = rospy.Time.now()
            out.name = names
            out.position = [float(low.motor_state[G1_MOTOR_INDEX[n]].q) for n in names]
            out.velocity = [float(low.motor_state[G1_MOTOR_INDEX[n]].dq) for n in names]
            out.effort = [float(low.motor_state[G1_MOTOR_INDEX[n]].tau_est) for n in names]
            pub.publish(out)
        rate.sleep()


def run_fake(pub, rate_hz):
    if not rospy.get_param("~allow_fake_zeros", False):
        rospy.logfatal("[joint_bridge] mode=fake publishes ZERO joint angles and would silently corrupt the "
                       "heading. Start with _allow_fake_zeros:=true to confirm this is a bench test.")
        sys.exit(2)
    names = list(G1_MOTOR_INDEX)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        rospy.logwarn_throttle(5.0, "[joint_bridge] FAKE ZERO JOINT STATES: heading is wrong by the real waist angle")
        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name, out.position = names, [0.0] * len(names)
        pub.publish(out)
        rate.sleep()


def main():
    rospy.init_node("g1_joint_state_bridge")
    mode = rospy.get_param("~mode", "relay")
    required = rospy.get_param("~required_joint", "waist_yaw_joint")
    rate_hz = float(rospy.get_param("~rate", 50.0))
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    if mode == "relay":
        run_relay(pub, required, rate_hz, float(rospy.get_param("~max_age", 1.0)))
    elif mode == "dds":
        run_dds(pub, required, rate_hz)
    elif mode == "fake":
        run_fake(pub, rate_hz)
    else:
        rospy.logfatal("[joint_bridge] unknown ~mode '%s' (relay|dds|fake)", mode)
        sys.exit(2)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
