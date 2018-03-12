# Install script for directory: /home/ahmed/tmp/civic-hack/catkin_ws_civic/src/panda_bridge_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ahmed/tmp/civic-hack/catkin_ws_civic/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ahmed/tmp/civic-hack/catkin_ws_civic/build/panda_bridge_ros/catkin_generated/installspace/panda_bridge_ros.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/panda_bridge_ros/cmake" TYPE FILE FILES
    "/home/ahmed/tmp/civic-hack/catkin_ws_civic/build/panda_bridge_ros/catkin_generated/installspace/panda_bridge_rosConfig.cmake"
    "/home/ahmed/tmp/civic-hack/catkin_ws_civic/build/panda_bridge_ros/catkin_generated/installspace/panda_bridge_rosConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/panda_bridge_ros" TYPE FILE FILES "/home/ahmed/tmp/civic-hack/catkin_ws_civic/src/panda_bridge_ros/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/panda_bridge_ros" TYPE PROGRAM FILES
    "/home/ahmed/tmp/civic-hack/catkin_ws_civic/src/panda_bridge_ros/scripts/frame_decoder.py"
    "/home/ahmed/tmp/civic-hack/catkin_ws_civic/src/panda_bridge_ros/scritps/panda_bridge_ros.py"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/panda_bridge_ros" TYPE FILE FILES "/home/ahmed/tmp/civic-hack/catkin_ws_civic/src/panda_bridge_ros/config/honda_civic_touring_2016_can_for_cantools.dbc")
endif()

