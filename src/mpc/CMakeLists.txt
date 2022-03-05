cmake_minimum_required(VERSION 3.0.2)
project(mpc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/mpc_double_shooting_obstacle_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
