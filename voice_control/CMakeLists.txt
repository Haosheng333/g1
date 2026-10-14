cmake_minimum_required(VERSION 3.0.2)
project(g1_voice_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/voice_control_node.py
  scripts/list_microphones.py
  scripts/recognize_wav.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # Pure in-process tests: no ROS master needed.
  catkin_add_nosetests(test/test_commands.py)
  catkin_add_nosetests(test/test_dispatch.py)
  catkin_add_nosetests(test/test_orchestrator_client.py)

  # Real ROS transport test: needs a roscore, kept separate from the above.
  find_package(rostest REQUIRED)
  add_rostest(test/test_ros_transport.test)
endif()
