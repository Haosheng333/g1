cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  GetObjectPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/perception_node.py
  scripts/perception_viewer.py
  scripts/demo_m1.py
  test/fake_camera_publisher.py
  scripts/stop_perception.sh
  scripts/start_remote_camera.sh
  test/desk_level_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_m1_perception.py)
  catkin_add_nosetests(test/test_node_helpers.py)
  catkin_add_nosetests(test/test_overlay.py)
  catkin_add_nosetests(test/test_cylinder_center.py)
  catkin_add_nosetests(test/test_measured_size.py)
  catkin_add_nosetests(test/test_lazy_import.py)
endif()
