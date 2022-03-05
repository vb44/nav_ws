###############################################################################
# set minimum required version of cmake, project name and compile options
###############################################################################
cmake_minimum_required(VERSION 3.0.2)
project(slam)

###############################################################################
# find catkin packages and libraries for catkin and system dependencies
###############################################################################
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
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
catkin_install_python(PROGRAMS src/hector_pose_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)