cmake_minimum_required(VERSION 3.0.2)
project(g1_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  NavigateToObject.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/navigation_node.py
  scripts/cmd_vel_gate.py
  scripts/obstacle_cloud_node.py
  scripts/g1_joint_state_bridge.py
  scripts/sim_world.py
  scripts/send_goal_node.py
  scripts/send_waypoints_node.py
  scripts/nav_client.py
  scripts/semantic_goal.py
  scripts/costmap.py
  scripts/global_planner.py
  scripts/local_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz sim
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
