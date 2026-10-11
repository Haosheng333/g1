#!/usr/bin/env python3
"""Send ONE pose goal to /move_base_simple/goal and wait for the result.

    rosrun g1_navigation send_goal_node.py _x:=1.0 _y:=1.0 _yaw:=0.0
Coordinates are in the z-UP `map` frame (_frame:=map).
"""
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

from nav_client import StatusWatcher


def main():
    rospy.init_node("send_goal_node", anonymous=True)
    x = float(rospy.get_param("~x", 1.0))
    y = float(rospy.get_param("~y", 1.0))
    yaw = float(rospy.get_param("~yaw", 0.0))
    frame = rospy.get_param("~frame", "map")
    timeout = float(rospy.get_param("~timeout", 600.0))

    watcher = StatusWatcher()
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)  # let the publisher connect

    goal = PoseStamped()
    goal.header.frame_id = frame
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x, goal.pose.position.y = x, y
    q = quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q
    rospy.loginfo("[send_goal_node] goal x=%.2f y=%.2f yaw=%.2f frame=%s", x, y, yaw, frame)
    pub.publish(goal)

    result = watcher.wait_terminal(timeout)
    rospy.loginfo("[send_goal_node] result: %s", result)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
