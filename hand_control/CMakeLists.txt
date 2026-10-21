cmake_minimum_required(VERSION 3.0.2)
project(hand_control)

find_package(catkin REQUIRED COMPONENTS rospy std_srvs)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/hand_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Starts g1_intellect's g1_hands Revo2 nodes on PC2 over SSH (see README).
install(PROGRAMS scripts/start_remote_hands.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
