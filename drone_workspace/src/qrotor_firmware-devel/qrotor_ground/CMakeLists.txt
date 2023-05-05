cmake_minimum_required(VERSION 2.8.3)
project(qrotor_ground)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        std_msgs
        qrotor_firmware
        roscpp
        rospy
        dynamic_reconfigure
        eigen_conversions
        )

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Trajectories.cfg
  cfg/DroneCommands.cfg
  cfg/MultiDroneManager.cfg
  cfg/Catenary.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES qrotor_ground
        CATKIN_DEPENDS geometry_msgs nav_msgs qrotor_firmware roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(QROTOR_GROUND_SRCS
        src/ground_station.cpp
        src/drone_manager.cpp
        src/nodes/qrotor_ground_node.cpp)

add_executable(ground_station_node ${QROTOR_GROUND_SRCS})
add_dependencies(ground_station_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_station_node ${catkin_LIBRARIES})

#add_executable(mission_status src/qrotor_display.cpp src/nodes/qrotor_display_node.cpp)
#add_dependencies(mission_status ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mission_status ${catkin_LIBRARIES})

