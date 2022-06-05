project(ip_basic)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    sensor_msgs
)

catkin_package(
)

catkin_install_python(PROGRAMS scripts/ip_basic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

