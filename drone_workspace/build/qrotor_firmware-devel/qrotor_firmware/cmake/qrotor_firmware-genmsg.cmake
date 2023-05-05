# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "qrotor_firmware: 3 messages, 2 services")

set(MSG_I_FLAGS "-Iqrotor_firmware:/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(qrotor_firmware_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg" NAME_WE)
add_custom_target(_qrotor_firmware_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qrotor_firmware" "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg" NAME_WE)
add_custom_target(_qrotor_firmware_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qrotor_firmware" "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg" NAME_WE)
add_custom_target(_qrotor_firmware_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qrotor_firmware" "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg" "std_msgs/Header:geometry_msgs/Vector3:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv" NAME_WE)
add_custom_target(_qrotor_firmware_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qrotor_firmware" "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv" ""
)

get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv" NAME_WE)
add_custom_target(_qrotor_firmware_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qrotor_firmware" "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv" "geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_cpp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_cpp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qrotor_firmware
)

### Generating Services
_generate_srv_cpp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qrotor_firmware
)
_generate_srv_cpp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qrotor_firmware
)

### Generating Module File
_generate_module_cpp(qrotor_firmware
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qrotor_firmware
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(qrotor_firmware_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(qrotor_firmware_generate_messages qrotor_firmware_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_cpp _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_cpp _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_cpp _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_cpp _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_cpp _qrotor_firmware_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qrotor_firmware_gencpp)
add_dependencies(qrotor_firmware_gencpp qrotor_firmware_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qrotor_firmware_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_eus(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_eus(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qrotor_firmware
)

### Generating Services
_generate_srv_eus(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qrotor_firmware
)
_generate_srv_eus(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qrotor_firmware
)

### Generating Module File
_generate_module_eus(qrotor_firmware
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qrotor_firmware
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(qrotor_firmware_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(qrotor_firmware_generate_messages qrotor_firmware_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_eus _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_eus _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_eus _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_eus _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_eus _qrotor_firmware_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qrotor_firmware_geneus)
add_dependencies(qrotor_firmware_geneus qrotor_firmware_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qrotor_firmware_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_lisp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_lisp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qrotor_firmware
)

### Generating Services
_generate_srv_lisp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qrotor_firmware
)
_generate_srv_lisp(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qrotor_firmware
)

### Generating Module File
_generate_module_lisp(qrotor_firmware
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qrotor_firmware
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(qrotor_firmware_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(qrotor_firmware_generate_messages qrotor_firmware_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_lisp _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_lisp _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_lisp _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_lisp _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_lisp _qrotor_firmware_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qrotor_firmware_genlisp)
add_dependencies(qrotor_firmware_genlisp qrotor_firmware_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qrotor_firmware_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_nodejs(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_nodejs(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qrotor_firmware
)

### Generating Services
_generate_srv_nodejs(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qrotor_firmware
)
_generate_srv_nodejs(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qrotor_firmware
)

### Generating Module File
_generate_module_nodejs(qrotor_firmware
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qrotor_firmware
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(qrotor_firmware_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(qrotor_firmware_generate_messages qrotor_firmware_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_nodejs _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_nodejs _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_nodejs _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_nodejs _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_nodejs _qrotor_firmware_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qrotor_firmware_gennodejs)
add_dependencies(qrotor_firmware_gennodejs qrotor_firmware_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qrotor_firmware_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_py(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware
)
_generate_msg_py(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware
)

### Generating Services
_generate_srv_py(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware
)
_generate_srv_py(qrotor_firmware
  "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware
)

### Generating Module File
_generate_module_py(qrotor_firmware
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(qrotor_firmware_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(qrotor_firmware_generate_messages qrotor_firmware_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_py _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_py _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_py _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_py _qrotor_firmware_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv" NAME_WE)
add_dependencies(qrotor_firmware_generate_messages_py _qrotor_firmware_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qrotor_firmware_genpy)
add_dependencies(qrotor_firmware_genpy qrotor_firmware_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qrotor_firmware_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qrotor_firmware)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qrotor_firmware
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(qrotor_firmware_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(qrotor_firmware_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qrotor_firmware)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qrotor_firmware
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(qrotor_firmware_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(qrotor_firmware_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qrotor_firmware)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qrotor_firmware
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(qrotor_firmware_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(qrotor_firmware_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qrotor_firmware)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qrotor_firmware
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(qrotor_firmware_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(qrotor_firmware_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qrotor_firmware
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(qrotor_firmware_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(qrotor_firmware_generate_messages_py std_msgs_generate_messages_py)
endif()
