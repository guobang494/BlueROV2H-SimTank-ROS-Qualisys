cmake_minimum_required(VERSION 3.0.2)
project(simulation_to_real_bridge)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  uuv_gazebo_ros_plugins_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    gazebo_msgs
    uuv_gazebo_ros_plugins_msgs
    tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

################################################
## Install ##
################################################

## Install Python scripts (nodes)
catkin_install_python(PROGRAMS
  src/sim_to_real_bridge.py
  src/thruster_allocator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files (YAML)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
