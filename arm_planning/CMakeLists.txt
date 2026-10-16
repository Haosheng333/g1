cmake_minimum_required(VERSION 3.0.2)
project(arm_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  PlanToPose.srv
  PlanGraspFromPoint.srv
  ConvertObservation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/arm_planning_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Runtime assets resolved with rospkg by arm_planning_node.py (never cwd).
# meshes/ and models/ hold large binaries and are installed as-is.
install(DIRECTORY config collision_masks meshes models urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
