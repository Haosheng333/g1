cmake_minimum_required(VERSION 3.0.2)
project(orchestrator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  RunRoute.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs sensor_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/orchestrator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
