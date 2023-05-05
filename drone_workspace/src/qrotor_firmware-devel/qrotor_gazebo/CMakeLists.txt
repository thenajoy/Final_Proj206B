cmake_minimum_required(VERSION 2.8.3)
project(qrotor_gazebo)

find_package(gazebo)

add_compile_options(-std=c++11)
# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_plugins
        gazebo_ros
        geometry_msgs
        sensor_msgs
        std_msgs
        qrotor_firmware
        ddynamic_reconfigure
        tf2
        tf2_ros
        tf2_geometry_msgs
        eigen_conversions
        )

find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp gazebo_plugins gazebo_ros geometry_msgs std_msgs qrotor_firmware tf2 tf2_ros sensor_msgs
        DEPENDS EIGEN3 GAZEBO
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

#set(QROTOR_SIL_SRCS
#        src/multirotor_forces_moments.cpp
#        src/qrotor_sil.cpp
#        src/sil_board.cpp
#        src/tracking_camera.cpp)

# qrotor_sil_plugin
#add_library(qrotor_sil_plugin SHARED ${QROTOR_SIL_SRCS})
#target_link_libraries(qrotor_sil_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} qrotor_firmware pose_estimation)
#add_dependencies(qrotor_sil_plugin ${catkin_EXPORTED_TARGETS})

# quadrotor_sil_plugin
set(QUADROTOR_SIL_SRCS
        src/quadrotor_sil.cpp
        src/sil_board.cpp
        src/multirotor_forces_moments.cpp
        )
add_library(quadrotor_sil_plugin SHARED ${QUADROTOR_SIL_SRCS})
target_link_libraries(quadrotor_sil_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} qrotor_firmware firmware_params_reconfig)
add_dependencies(quadrotor_sil_plugin ${catkin_EXPORTED_TARGETS})

# rigidbody_plugin
#add_library(rigidbody_plugin SHARED src/rigidbody_plugin.cpp)
#target_link_libraries(rigidbody_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(rigidbody_plugin ${catkin_EXPORTED_TARGETS})

# multiquad_payload_plugin
#add_library(multiquad_payload_plugin SHARED src/multiquad_payload_plugin.cpp)
#target_link_libraries(multiquad_payload_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(multiquad_payload_plugin ${catkin_EXPORTED_TARGETS})

# model example plugin
#add_library(model_example_plugin SHARED src/model_example_plugin.cpp)
#target_link_libraries(model_example_plugin ${GAZEBO_LIBRARIES})

add_executable(qrotor_fake_rc_node src/qrotor_fake_rc.cpp)
add_dependencies(qrotor_fake_rc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qrotor_fake_rc_node ${catkin_LIBRARIES})

add_executable(multi_qrotor_fake_rc_node src/multi_qrotor_fake_rc.cpp)
add_dependencies(multi_qrotor_fake_rc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_qrotor_fake_rc_node ${catkin_LIBRARIES})

