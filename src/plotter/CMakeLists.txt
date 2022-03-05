###############################################################################
# set minimum required version of cmake, project name and compile options
###############################################################################
cmake_minimum_required(VERSION 3.0.2)
project(plotter)

###############################################################################
# find catkin packages and libraries for catkin and system dependencies
###############################################################################
find_package(catkin REQUIRED
    rospy
)

###############################################################################
# declare catkin specific configuration to be passed to dependent projects
###############################################################################
catkin_package(
)

include_directories(
)

###############################################################################
# install
###############################################################################
catkin_install_python(PROGRAMS src/plotter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)