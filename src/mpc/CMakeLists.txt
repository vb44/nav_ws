###############################################################################
# set minimum required version of cmake, project name and compile options
###############################################################################
cmake_minimum_required(VERSION 3.0.2)
project(mpc)

###############################################################################
# find catkin packages and libraries for catkin and system dependencies
###############################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
)

###############################################################################
# declare catkin specific configuration to be passed to dependent projects
###############################################################################
catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

###############################################################################
# install
###############################################################################
catkin_install_python(PROGRAMS src/mpc_double_shooting_obstacle_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)