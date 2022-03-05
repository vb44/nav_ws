###############################################################################
# set minimum required version of cmake, project name and compile options
###############################################################################
cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

###############################################################################
# find catkin packages and libraries for catkin and system dependencies
###############################################################################
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

###############################################################################
# declare ROS messages, services and actions
###############################################################################
add_message_files(
  FILES
  Cluster.msg
  Predict.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###############################################################################
# declare catkin specific configuration to be passed to dependent projects
###############################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)
