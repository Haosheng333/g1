#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

def move_to_goal(x, y, yaw):
    rospy.init_node('send_goal_node', anonymous=True)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server!")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    goal.target_pose.pose.position.z = 0.0

    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    rospy.loginfo(f"Sending goal: x={x}, y={y}, yaw={yaw}")
    client.send_goal(goal)

    finished_in_time = client.wait_for_result(rospy.Duration(60.0))

    if not finished_in_time:
        client.cancel_goal()
        rospy.logerr("Timeout: Robot took too long to reach target.")
    else:
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Target reached successfully!")
        else:
            rospy.logerr(f"Goal failed with status code: {state}")

if __name__ == '__main__':
    try:
        move_to_goal(1.0, 1.0, 0.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node interrupted.")
