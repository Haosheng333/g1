#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

# Define sequence of waypoints: (x, y, yaw in radians)
WAYPOINTS = [
    (1.0, 0.0, 0.0),      # Waypoint 1
    (1.0, 1.0, 1.57),     # Waypoint 2
    (0.0, 1.0, 3.14),     # Waypoint 3
    (0.0, 0.0, 0.0)       # Return to start
]

def execute_waypoints():
    rospy.init_node('send_waypoints_node', anonymous=True)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    rospy.loginfo("Connecting to move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server!")

    for i, (x, y, yaw) in enumerate(WAYPOINTS, start=1):
        if rospy.is_shutdown():
            break

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

        rospy.loginfo(f"[Waypoint {i}/{len(WAYPOINTS)}] Sending goal: x={x}, y={y}, yaw={yaw}")
        client.send_goal(goal)

        # Wait for robot to reach waypoint before proceeding to next
        finished_in_time = client.wait_for_result(rospy.Duration(60.0))

        if not finished_in_time:
            client.cancel_goal()
            rospy.logerr(f"[Waypoint {i}] Timeout reached. Skipping to next waypoint.")
        else:
            state = client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"[Waypoint {i}] Reached successfully!")
            else:
                rospy.logwarn(f"[Waypoint {i}] Failed with status code: {state}")
        
        rospy.sleep(1.0) # Brief pause between waypoints

    rospy.loginfo("Waypoint navigation mission complete!")

if __name__ == '__main__':
    try:
        execute_waypoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint navigation interrupted.")
