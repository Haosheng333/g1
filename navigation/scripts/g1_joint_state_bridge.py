#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def publish_joint_states():
    rospy.init_node('g1_joint_state_bridge', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(50)

    # Complete joint set for Unitree G1 (urdf/g1_23dof_mode_10.urdf)
    joint_names = [
        'waist_yaw_joint',
        'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
        'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
        'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
        'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
        'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
        'left_elbow_joint', 'left_wrist_roll_joint',
        'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
        'right_elbow_joint', 'right_wrist_roll_joint'
    ]

    rospy.loginfo("[g1_joint_state_bridge] Live joint state bridge active.")

    while not rospy.is_shutdown():
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = joint_names
        js.position = [0.0] * len(joint_names)
        js.velocity = []
        js.effort = []
        pub.publish(js)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
