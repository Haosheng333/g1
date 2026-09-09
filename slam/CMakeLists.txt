cmake_minimum_required(VERSION 3.0.2)
project(g1_slam)

find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
  roslaunch
  sensor_msgs
  nav_msgs
)

# libpcl-dev / pcl_ros provide these; used only by the offline pcd_to_map tool.
find_package(PCL REQUIRED COMPONENTS common io)

# Installs the g1_slam Python package from src/g1_slam (see setup.py) so that
# nodes and tests can `import g1_slam.tf_chain`. MUST come before
# catkin_package().
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/slam_health_check.py
  scripts/save_map.py
  scripts/g1_tf_publisher.py
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

# Vendored official robot description (URDF + upstream LICENSE + SOURCE.md).
# g1_tf.launch loads it with textfile="$(find g1_slam)/urdf/...", so it must be
# installed alongside launch/ and config/.
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

if(CATKIN_ENABLE_TESTING)
  # Transform-direction and URDF-agreement tests. nose, not pytest - see the
  # python3-nose test_depend in package.xml.
  catkin_add_nosetests(test)
endif()
