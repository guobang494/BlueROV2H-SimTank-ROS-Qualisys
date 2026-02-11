# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bluerov2_control: 9 messages, 2 services")

set(MSG_I_FLAGS "-Ibluerov2_control:/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg;-Ibluerov2_control:/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg;-Igeographic_msgs:/opt/ros/noetic/share/geographic_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iuuv_control_msgs:/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iuuid_msgs:/opt/ros/noetic/share/uuid_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bluerov2_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg" "std_msgs/Header"
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg" "std_msgs/Time:bluerov2_control/FollowWaypointsActionResult:geometry_msgs/Point:uuv_control_msgs/WaypointSet:bluerov2_control/FollowWaypointsGoal:uuv_control_msgs/Waypoint:bluerov2_control/FollowWaypointsFeedback:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:bluerov2_control/FollowWaypointsResult:bluerov2_control/FollowWaypointsActionGoal:bluerov2_control/FollowWaypointsActionFeedback:std_msgs/Header"
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg" "std_msgs/Time:geometry_msgs/Point:uuv_control_msgs/WaypointSet:bluerov2_control/FollowWaypointsGoal:uuv_control_msgs/Waypoint:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:bluerov2_control/FollowWaypointsResult:std_msgs/Header"
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg" "bluerov2_control/FollowWaypointsFeedback:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg" "std_msgs/Header:std_msgs/Time:uuv_control_msgs/WaypointSet:uuv_control_msgs/Waypoint:geometry_msgs/Point"
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv" "geometry_msgs/Point:geographic_msgs/GeoPoint"
)

get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv" NAME_WE)
add_custom_target(_bluerov2_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluerov2_control" "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv" "bluerov2_control/ControlMode"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)

### Generating Services
_generate_srv_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)
_generate_srv_cpp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
)

### Generating Module File
_generate_module_cpp(bluerov2_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bluerov2_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bluerov2_control_generate_messages bluerov2_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_cpp _bluerov2_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_gencpp)
add_dependencies(bluerov2_control_gencpp bluerov2_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_msg_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)

### Generating Services
_generate_srv_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)
_generate_srv_eus(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
)

### Generating Module File
_generate_module_eus(bluerov2_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bluerov2_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bluerov2_control_generate_messages bluerov2_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_eus _bluerov2_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_geneus)
add_dependencies(bluerov2_control_geneus bluerov2_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_msg_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)

### Generating Services
_generate_srv_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)
_generate_srv_lisp(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
)

### Generating Module File
_generate_module_lisp(bluerov2_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bluerov2_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bluerov2_control_generate_messages bluerov2_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_lisp _bluerov2_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_genlisp)
add_dependencies(bluerov2_control_genlisp bluerov2_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_msg_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)

### Generating Services
_generate_srv_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)
_generate_srv_nodejs(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
)

### Generating Module File
_generate_module_nodejs(bluerov2_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(bluerov2_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(bluerov2_control_generate_messages bluerov2_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_nodejs _bluerov2_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_gennodejs)
add_dependencies(bluerov2_control_gennodejs bluerov2_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Time.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/WaypointSet.msg;/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_msg_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)

### Generating Services
_generate_srv_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geographic_msgs/cmake/../msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)
_generate_srv_py(bluerov2_control
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
)

### Generating Module File
_generate_module_py(bluerov2_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bluerov2_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bluerov2_control_generate_messages bluerov2_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv" NAME_WE)
add_dependencies(bluerov2_control_generate_messages_py _bluerov2_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluerov2_control_genpy)
add_dependencies(bluerov2_control_genpy bluerov2_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluerov2_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluerov2_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_control_generate_messages_cpp geographic_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET uuv_control_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_control_generate_messages_cpp uuv_control_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(bluerov2_control_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bluerov2_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_eus)
  add_dependencies(bluerov2_control_generate_messages_eus geographic_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(bluerov2_control_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET uuv_control_msgs_generate_messages_eus)
  add_dependencies(bluerov2_control_generate_messages_eus uuv_control_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(bluerov2_control_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluerov2_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_control_generate_messages_lisp geographic_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET uuv_control_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_control_generate_messages_lisp uuv_control_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(bluerov2_control_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bluerov2_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_control_generate_messages_nodejs geographic_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_control_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET uuv_control_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_control_generate_messages_nodejs uuv_control_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(bluerov2_control_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluerov2_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_py)
  add_dependencies(bluerov2_control_generate_messages_py geographic_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(bluerov2_control_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET uuv_control_msgs_generate_messages_py)
  add_dependencies(bluerov2_control_generate_messages_py uuv_control_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(bluerov2_control_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
