# Install script for directory: /root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_assistants

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
  include("/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/safe_execute_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/uuv_assistants.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_assistants/cmake" TYPE FILE FILES
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/uuv_assistantsConfig.cmake"
    "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/uuv_assistantsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_assistants" TYPE FILE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_assistants/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/publish_footprints.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/set_simulation_timer.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/publish_world_models.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/publish_vehicle_footprint.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/unpause_simulation.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/create_new_robot_model")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants" TYPE PROGRAM FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/build/uuv_simulator/uuv_assistants/catkin_generated/installspace/create_thruster_manager_configuration")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_assistants" TYPE DIRECTORY FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/src/uuv_simulator/uuv_assistants/launch" REGEX "/[^/]*\\~$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants/uuv_message_to_tf" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants/uuv_message_to_tf")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants/uuv_message_to_tf"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants" TYPE EXECUTABLE FILES "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/devel/lib/uuv_assistants/uuv_message_to_tf")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants/uuv_message_to_tf" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants/uuv_message_to_tf")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants/uuv_message_to_tf"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uuv_assistants/uuv_message_to_tf")
    endif()
  endif()
endif()

