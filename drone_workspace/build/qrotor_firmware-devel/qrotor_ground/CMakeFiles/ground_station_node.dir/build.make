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

# Include any dependencies generated for this target.
include qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/depend.make

# Include the progress variables for this target.
include qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/progress.make

# Include the compile flags for this target's objects.
include qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/flags.make

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/ground_station.cpp.o: qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/flags.make
qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/ground_station.cpp.o: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/ground_station.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/ground_station.cpp.o"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ground_station_node.dir/src/ground_station.cpp.o -c /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/ground_station.cpp

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/ground_station.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ground_station_node.dir/src/ground_station.cpp.i"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/ground_station.cpp > CMakeFiles/ground_station_node.dir/src/ground_station.cpp.i

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/ground_station.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ground_station_node.dir/src/ground_station.cpp.s"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/ground_station.cpp -o CMakeFiles/ground_station_node.dir/src/ground_station.cpp.s

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.o: qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/flags.make
qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.o: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/drone_manager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.o"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.o -c /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/drone_manager.cpp

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.i"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/drone_manager.cpp > CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.i

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.s"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/drone_manager.cpp -o CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.s

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.o: qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/flags.make
qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.o: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/nodes/qrotor_ground_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.o"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.o -c /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/nodes/qrotor_ground_node.cpp

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.i"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/nodes/qrotor_ground_node.cpp > CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.i

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.s"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/src/nodes/qrotor_ground_node.cpp -o CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.s

# Object files for target ground_station_node
ground_station_node_OBJECTS = \
"CMakeFiles/ground_station_node.dir/src/ground_station.cpp.o" \
"CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.o" \
"CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.o"

# External object files for target ground_station_node
ground_station_node_EXTERNAL_OBJECTS =

/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/ground_station.cpp.o
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/drone_manager.cpp.o
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/src/nodes/qrotor_ground_node.cpp.o
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/build.make
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/libddynamic_reconfigure.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/libroscpp.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/librosconsole.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/libeigen_conversions.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/liborocos-kdl.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/librostime.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /opt/ros/noetic/lib/libcpp_common.so
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node: qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node"
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ground_station_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/build: /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/devel/lib/qrotor_ground/ground_station_node

.PHONY : qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/build

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/clean:
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground && $(CMAKE_COMMAND) -P CMakeFiles/ground_station_node.dir/cmake_clean.cmake
.PHONY : qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/clean

qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/depend:
	cd /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground /home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/build/qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : qrotor_firmware-devel/qrotor_ground/CMakeFiles/ground_station_node.dir/depend
