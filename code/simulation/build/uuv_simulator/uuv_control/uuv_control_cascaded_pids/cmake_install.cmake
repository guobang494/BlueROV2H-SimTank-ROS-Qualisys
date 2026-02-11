# Install script for directory: /root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids

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
  include("/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/safe_execute_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uuv_control_cascaded_pid" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/include/uuv_control_cascaded_pid/AccelerationControlConfig.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uuv_control_cascaded_pid" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/include/uuv_control_cascaded_pid/PositionControlConfig.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uuv_control_cascaded_pid" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/include/uuv_control_cascaded_pid/VelocityControlConfig.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/uuv_control_cascaded_pid" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/python3/dist-packages/uuv_control_cascaded_pid/__init__.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/python3/dist-packages/uuv_control_cascaded_pid/cfg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/uuv_control_cascaded_pid" TYPE DIRECTORY FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/python3/dist-packages/uuv_control_cascaded_pid/cfg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/uuv_control_cascaded_pid.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_cascaded_pid/cmake" TYPE FILE FILES
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/uuv_control_cascaded_pidConfig.cmake"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/uuv_control_cascaded_pidConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_cascaded_pid" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_control_cascaded_pid" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/AccelerationControl.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_control_cascaded_pid" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/PositionControl.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_control_cascaded_pid" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/VelocityControl.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_cascaded_pid" TYPE DIRECTORY FILES
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/launch"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/config"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/cfg"
    REGEX "/[^/]*\\~$" EXCLUDE)
endif()

