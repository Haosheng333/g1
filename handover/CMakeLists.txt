cmake_minimum_required(VERSION 3.0.2)
project(handover)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  ArmMonitor.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/auto_release_monitor.py
  scripts/record_tactile.py
  scripts/handover_node.py
  scripts/handover_cli.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
