#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def send_goal(x, y, yaw):
    rospy.init_node('send_goal_node', anonymous=True)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    
    rospy.sleep(0.5) # Allow publisher connection time

    goal = PoseStamped()
    goal.header.frame_id = rospy.get_param('~global_frame', 'camera_init')
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.position.z = 0.0

    q = quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    rospy.loginfo(f"[send_goal_node] Publishing goal to /move_base_simple/goal: x={x}, y={y}, frame={goal.header.frame_id}")
    goal_pub.publish(goal)
    rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        send_goal(1.0, 1.0, 0.0)
    except rospy.ROSInterruptException:
        pass
