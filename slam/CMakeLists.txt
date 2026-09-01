cmake_minimum_required(VERSION 3.0.2)
project(g1_slam)

find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
  roslaunch
  sensor_msgs
)

# libpcl-dev / pcl_ros provide these; used only by the offline pcd_to_map tool.
find_package(PCL REQUIRED COMPONENTS common io)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/slam_health_check.py
  scripts/save_map.py
  scripts/g1_static_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Offline PCD -> 2D occupancy grid converter (FAST-LIO2 binary PCD -> map_server).
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(pcd_to_map src/pcd_to_map.cpp)
target_link_libraries(pcd_to_map
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

install(TARGETS pcd_to_map
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
