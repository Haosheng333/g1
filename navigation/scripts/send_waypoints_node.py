#!/usr/bin/env python3

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

WAYPOINTS = [
    (1.0, 0.0, 0.0),
    (1.0, 1.0, 1.57),
    (0.0, 1.0, 3.14),
    (0.0, 0.0, 0.0)
]

def execute_waypoints():
    rospy.init_node('send_waypoints_node', anonymous=True)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    tf_listener = tf.TransformListener()

    global_frame = rospy.get_param('~global_frame', 'camera_init')
    robot_frame = rospy.get_param('~robot_frame', 'pelvis')

    for i, (x, y, yaw) in enumerate(WAYPOINTS, start=1):
        if rospy.is_shutdown():
            break

        goal = PoseStamped()
        goal.header.frame_id = global_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)

        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q

        rospy.loginfo(f"[Waypoint {i}/{len(WAYPOINTS)}] Sending goal: x={x}, y={y}")
        goal_pub.publish(goal)

        # Wait until robot approaches waypoint
        reached = False
        while not rospy.is_shutdown() and not reached:
            try:
                (trans, _) = tf_listener.lookupTransform(global_frame, robot_frame, rospy.Time(0))
                dist = math.hypot(x - trans[0], y - trans[1])
                if dist < 0.25:
                    rospy.loginfo(f"[Waypoint {i}] Reached target proximity!")
                    reached = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            rospy.sleep(0.2)

        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        execute_waypoints()
    except rospy.ROSInterruptException:
        pass
