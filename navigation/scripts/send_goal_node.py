#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_nav_goal(x, y, yaw=0.0):
    rospy.init_node('g1_navigation_client')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # Simple orientation

    rospy.loginfo(f"Sending navigation goal: x={x}, y={y}")
    client.send_goal(goal)
    client.wait_for_result()
    
    return client.get_result()

if __name__ == '__main__':
    try:
        send_nav_goal(1.0, 2.0)
    except rospy.ROSInterruptException:
        pass