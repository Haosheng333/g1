cmake_minimum_required(VERSION 3.0.2)
project(g1_slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/slam_health_check.py
  scripts/save_map.py
  scripts/g1_static_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)
