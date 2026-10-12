#!/usr/bin/env python3
"""Drive a fixed list of pose goals one after another (bench test).

Waits for /navigation/status SUCCESS (with the saw_active guard) instead of
polling TF distance, and stops on the first FAILURE / TIMEOUT.
Waypoints are (x, y, yaw) in the z-UP `map` frame - edit WAYPOINTS to suit the room.
"""
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

from nav_client import StatusWatcher

WAYPOINTS = [
    (1.0, 0.0, 0.0),
    (1.0, 1.0, 1.57),
    (0.0, 1.0, 3.14),
    (0.0, 0.0, 0.0),
]


def execute_waypoints():
    rospy.init_node("send_waypoints_node", anonymous=True)
    frame = rospy.get_param("~frame", "map")
    timeout = float(rospy.get_param("~timeout", 600.0))
    watcher = StatusWatcher()
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)

    for i, (x, y, yaw) in enumerate(WAYPOINTS, start=1):
        if rospy.is_shutdown():
            return
        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q
        rospy.loginfo("[Waypoint %d/%d] x=%.2f y=%.2f yaw=%.2f", i, len(WAYPOINTS), x, y, yaw)
        watcher.latest = None            # forget the previous goal's terminal state
        pub.publish(goal)
        result = watcher.wait_terminal(timeout)
        rospy.loginfo("[Waypoint %d/%d] %s", i, len(WAYPOINTS), result)
        if result != "SUCCESS":
            rospy.logerr("stopping: waypoint %d ended with %s", i, result)
            return
        rospy.sleep(1.0)


if __name__ == "__main__":
    try:
        execute_waypoints()
    except rospy.ROSInterruptException:
        pass
