# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build

# Utility rule file for qrotor_firmware_generate_messages_eus.

# Include the progress variables for this target.
include qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/progress.make

qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/Log.l
qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/RCRaw.l
qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/AttitudeState.l
qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/Setpoint.l
qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/FlatTrajectory.l
qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/manifest.l


/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/Log.l: /opt/ros/noetic/lib/geneus/gen_eus.py
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/Log.l: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/Log.l: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/Log.l: /opt/ros/noetic/share/geometry_msgs/msg/Vector3.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating EusLisp code from qrotor_firmware/Log.msg"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/Log.msg -Iqrotor_firmware:/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p qrotor_firmware -o /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg

/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/RCRaw.l: /opt/ros/noetic/lib/geneus/gen_eus.py
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/RCRaw.l: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/RCRaw.l: /opt/ros/noetic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating EusLisp code from qrotor_firmware/RCRaw.msg"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/RCRaw.msg -Iqrotor_firmware:/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p qrotor_firmware -o /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg

/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/AttitudeState.l: /opt/ros/noetic/lib/geneus/gen_eus.py
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/AttitudeState.l: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/AttitudeState.l: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/AttitudeState.l: /opt/ros/noetic/share/geometry_msgs/msg/Vector3.msg
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/AttitudeState.l: /opt/ros/noetic/share/geometry_msgs/msg/Quaternion.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating EusLisp code from qrotor_firmware/AttitudeState.msg"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg/AttitudeState.msg -Iqrotor_firmware:/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p qrotor_firmware -o /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg

/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/Setpoint.l: /opt/ros/noetic/lib/geneus/gen_eus.py
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/Setpoint.l: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating EusLisp code from qrotor_firmware/Setpoint.srv"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/Setpoint.srv -Iqrotor_firmware:/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p qrotor_firmware -o /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv

/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/FlatTrajectory.l: /opt/ros/noetic/lib/geneus/gen_eus.py
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/FlatTrajectory.l: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/FlatTrajectory.l: /opt/ros/noetic/share/geometry_msgs/msg/Vector3.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating EusLisp code from qrotor_firmware/FlatTrajectory.srv"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/srv/FlatTrajectory.srv -Iqrotor_firmware:/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p qrotor_firmware -o /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv

/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/manifest.l: /opt/ros/noetic/lib/geneus/gen_eus.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating EusLisp manifest code for qrotor_firmware"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/geneus/cmake/../../../lib/geneus/gen_eus.py -m -o /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware qrotor_firmware geometry_msgs std_msgs

qrotor_firmware_generate_messages_eus: qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus
qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/Log.l
qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/RCRaw.l
qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/msg/AttitudeState.l
qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/Setpoint.l
qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/srv/FlatTrajectory.l
qrotor_firmware_generate_messages_eus: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/share/roseus/ros/qrotor_firmware/manifest.l
qrotor_firmware_generate_messages_eus: qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/build.make

.PHONY : qrotor_firmware_generate_messages_eus

# Rule to build all files generated by this target.
qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/build: qrotor_firmware_generate_messages_eus

.PHONY : qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/build

qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/clean:
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware && $(CMAKE_COMMAND) -P CMakeFiles/qrotor_firmware_generate_messages_eus.dir/cmake_clean.cmake
.PHONY : qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/clean

qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/depend:
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_firmware /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : qrotor_firmware-devel/qrotor_firmware/CMakeFiles/qrotor_firmware_generate_messages_eus.dir/depend
