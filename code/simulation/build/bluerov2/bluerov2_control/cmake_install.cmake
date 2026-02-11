# Install script for directory: /root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/safe_execute_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluerov2_control/msg" TYPE FILE FILES
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/ControlMode.msg"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/msg/Autopilot.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluerov2_control/srv" TYPE FILE FILES
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/ConvertGeoPoints.srv"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/srv/SetControlMode.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluerov2_control/action" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/action/FollowWaypoints.action")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluerov2_control/msg" TYPE FILE FILES
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsAction.msg"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionGoal.msg"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionResult.msg"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsActionFeedback.msg"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsGoal.msg"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsResult.msg"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/bluerov2_control/msg/FollowWaypointsFeedback.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluerov2_control/cmake" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/bluerov2_control-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/include/bluerov2_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/roseus/ros/bluerov2_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/common-lisp/ros/bluerov2_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/share/gennodejs/ros/bluerov2_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/python3/dist-packages/bluerov2_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/python3/dist-packages/bluerov2_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bluerov2_control" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/include/bluerov2_control/pid_reconfigConfig.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bluerov2_control" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/include/bluerov2_control/marker_reconfigConfig.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/bluerov2_control" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/python3/dist-packages/bluerov2_control/__init__.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/python3/dist-packages/bluerov2_control/cfg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/bluerov2_control" TYPE DIRECTORY FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/python3/dist-packages/bluerov2_control/cfg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/bluerov2_control.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluerov2_control/cmake" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/bluerov2_control-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluerov2_control/cmake" TYPE FILE FILES
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/bluerov2_controlConfig.cmake"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/bluerov2_controlConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluerov2_control" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/bluerov2/bluerov2_control/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bluerov2_control" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/set_gm_current.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bluerov2_control" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/user_mav")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bluerov2_control" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/video")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bluerov2_control" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/sitl.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bluerov2_control" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/bluerov_node.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bluerov2_control" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/bridge.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bluerov2_control" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/bluerov2/bluerov2_control/catkin_generated/installspace/node")
endif()

