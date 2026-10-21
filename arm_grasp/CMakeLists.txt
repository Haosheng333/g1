cmake_minimum_required(VERSION 3.0.2)
project(arm_grasp)

find_package(catkin REQUIRED COMPONENTS
  g1_camera
  rospy
  std_msgs
  std_srvs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  MoveArmJoints.srv
  InferGraspFromPoint.srv
  ExecuteGraspFromPoint.srv
  ExecuteAstarPathFromPoint.srv
  PlanGraspFromPoint.srv
  PlanToPose.srv
  GoHomeArm.srv
  PressElevatorButton.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    g1_camera
    rospy
    std_msgs
    std_srvs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/start_remote_hands.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/arm_node_abs.py
  scripts/fetch_weights.py
  scripts/grasp_point.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
