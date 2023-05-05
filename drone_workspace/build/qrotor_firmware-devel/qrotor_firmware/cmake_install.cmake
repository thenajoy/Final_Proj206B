# Install script for directory: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qrotor_firmware/msg" TYPE FILE FILES
    "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg"
    "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg"
    "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qrotor_firmware/srv" TYPE FILE FILES
    "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv"
    "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/qrotor_firmware" TYPE FILE FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/include/qrotor_firmware/motor_calibConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/qrotor_firmware" TYPE FILE FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/python3/dist-packages/qrotor_firmware/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/python3/dist-packages/qrotor_firmware/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/qrotor_firmware" TYPE DIRECTORY FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/python3/dist-packages/qrotor_firmware/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qrotor_firmware/cmake" TYPE FILE FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware/catkin_generated/installspace/qrotor_firmware-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/include/qrotor_firmware")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/common-lisp/ros/qrotor_firmware")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/gennodejs/ros/qrotor_firmware")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/python3/dist-packages/qrotor_firmware")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/python3/dist-packages/qrotor_firmware")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware/catkin_generated/installspace/qrotor_firmware.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qrotor_firmware/cmake" TYPE FILE FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware/catkin_generated/installspace/qrotor_firmware-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qrotor_firmware/cmake" TYPE FILE FILES
    "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware/catkin_generated/installspace/qrotor_firmwareConfig.cmake"
    "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware/catkin_generated/installspace/qrotor_firmwareConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qrotor_firmware" TYPE FILE FILES "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/package.xml")
endif()

