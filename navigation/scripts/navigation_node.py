#!/usr/bin/env python3

import rospy
import tf
import math
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from costmap import OccupancyCostmap
from global_planner import AStarPlanner
from local_planner import PurePursuitController

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=True)

        # Configurable frame alignment matching g1_slam contract
        self.global_frame = rospy.get_param('~global_frame', 'camera_init')
        self.robot_base_frame = rospy.get_param('~robot_frame', 'pelvis')

        self.costmap = OccupancyCostmap()
        self.planner = AStarPlanner()
        self.controller = PurePursuitController()

        self.tf_listener = tf.TransformListener()
        self.current_path = []
        self.goal_pose = None

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/custom_global_path', Path, queue_size=1)

        rospy.loginfo(f"[navigation_node] Initialized with target frames: {self.global_frame} -> {self.robot_base_frame}")

    def map_callback(self, msg):
        self.costmap.update_map(msg)

    def goal_callback(self, msg):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"[navigation_node] New goal received: {self.goal_pose}")
        self.replan()

    def get_robot_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.global_frame, self.robot_base_frame, rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            return (trans[0], trans[1], yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(5.0, f"[navigation_node] Waiting for TF ({self.global_frame} -> {self.robot_base_frame}). Ensure /joint_states carries waist_yaw_joint.")
            return None

    def replan(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            rospy.logwarn("[navigation_node] Cannot plan: Missing valid robot pose.")
            return

        if self.goal_pose is None:
            rospy.logwarn("[navigation_node] Cannot plan: Missing goal pose.")
            return

        if self.costmap.grid is None:
            rospy.logwarn("[navigation_node] Cannot plan: Costmap not initialized.")
            return

        self.current_path = self.planner.plan(self.costmap, robot_pose, self.goal_pose)
        if self.current_path:
            rospy.loginfo(f"[navigation_node] Path generated with {len(self.current_path)} points.")
            self.publish_path()
        else:
            rospy.logerr("[navigation_node] Failed to find valid path to goal!")

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()

        for x, y in self.current_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_path:
                robot_pose = self.get_robot_pose()
                if robot_pose:
                    cmd, goal_reached = self.controller.compute_cmd_vel(robot_pose, self.current_path)
                    self.cmd_pub.publish(cmd)

                    if goal_reached:
                        rospy.loginfo("[navigation_node] Target goal reached!")
                        self.current_path = []
                        self.goal_pose = None
            rate.sleep()

if __name__ == '__main__':
    try:
        node = NavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
