cmake_minimum_required(VERSION 2.8.3)
project(qrotor_firmware)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        std_msgs
        nav_msgs
        std_srvs
        roscpp
        rospy
        message_generation
        ddynamic_reconfigure
        dynamic_reconfigure
        tf2
        tf2_ros
        tf2_geometry_msgs
        eigen_conversions
        )

add_message_files(
        FILES
        Log.msg
        RCRaw.msg
        AttitudeState.msg
)

add_service_files(
        FILES
        Setpoint.srv
        FlatTrajectory.srv
)

generate_dynamic_reconfigure_options(
        cfg/motor_calib.cfg
)

generate_messages(
        DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
        INCLUDE_DIRS include external
        LIBRARIES
        CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime ddynamic_reconfigure
        #  DEPENDS system_lib
)


### FOR PROFILING
add_compile_options(-pg)
set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)
###


include_directories(
        include
        external
        external/Navio2/C++/Navio
        external/Navio2/C++/Navio/Common
        external/Navio2/C++/Navio/Navio2
        external/Navio2/C++/Navio/Navio+
        external/mathlib/math/filter
        ${catkin_INCLUDE_DIRS}
        src/nodes/
)

#////////////////////////////////////////////////////////
#/////////////////////// libraries //////////////////////
#////////////////////////////////////////////////////////
# nonlinear_controls
# add_subdirectory(external/nonlinear_controls)

# matlab codegen
set(MATLAB_CODEGEN_DIR external/matlab_codegen/qcqp)
file(GLOB MATLAB_CODEGEN_SRCS "${MATLAB_CODEGEN_DIR}/*.cpp")
file(GLOB MATLAB_CODEGEN_INCLUDE "${MATLAB_CODEGEN_DIR}/*.h")
add_library(matlab_codegen ${MATLAB_CODEGEN_SRCS})
target_include_directories(matlab_codegen PUBLIC . ${MATLAB_CODEGEN_DIR})


# libnavio
file(GLOB_RECURSE NAVIO_SRCS "external/Navio2/C++/Navio/Common/*.cpp" "external/Navio2/C++/Navio/Navio2/*.cpp")
file(GLOB_RECURSE NAVIO_INCLUDE "external/Navio2/C++/Navio/Common/*.h*" "external/Navio2/C++/Navio/Navio2/*.h*")
add_library(navio ${NAVIO_INCLUDE} ${NAVIO_SRCS})
target_include_directories(navio PUBLIC . include)
target_link_libraries(navio ${catkin_LIBRARIES})
add_dependencies(navio ${catkin_EXPORTED_TARGETS})

# libmath
file(GLOB MATH_SRCS "external/mathlib/math/filter/*.cpp")
file(GLOB MATH_INCLUDE "external/mathlib/math/filter/*.hpp")
add_library(mathlib ${MATH_INCLUDE} ${MATH_SRCS})
target_include_directories(mathlib PUBLIC . include)
target_link_libraries(mathlib ${catkin_LIBRARIES})
add_dependencies(mathlib ${catkin_EXPORTED_TARGETS})

#libeigen_estimation
set(EIGEN_ESTIMATION_SRCS
        src/estimation/eigen/invariant_ekf.cpp
        src/estimation/eigen/inekf.cpp)

set(MATLAB_ESTIMATION_SRCS
        src/estimation/matlab/invariant_ekf_m.cpp)

# libqrotor_firmware
#file(GLOB CONTROL_SRCS "src/control/*.cpp") # fix this
set(CONTROL_SRCS
        # attitude controls
        # src/control/attitude_control_fsm.cpp
        src/control/attitude_controller.cpp
        # src/control/attitude_geometric_clf_qp.cpp
        src/control/attitude_geometric_control.cpp
        src/control/euler_angle_ppid.cpp
        # src/control/attitude_vbl_lqr.cpp
        src/control/euler_pid.cpp
        src/control/attitude_control_px4.cpp
        # position controls
        src/control/position_controller.cpp
        src/control/position_pid.cpp
        # src/control/position_clf_qp.cpp
        # src/control/position_mpc.cpp
        # common
        src/control/control_math.cpp)

set(ESTIMATION_SRCS
        src/estimation/attitude_estimator.cpp
        src/estimation/ahrs_default.cpp
        src/estimation/mahony_rf.cpp
        src/estimation/pose_naive.cpp
        src/estimation/pose_estimator.cpp
        src/estimation/pose_iekf.cpp
        src/estimation/madgwick_ahrs.cpp)

set(PERIPHERALS_SRCS
        src/peripherals/rs_t265.cpp
        )

set(PLANNING_SRCS
        src/planning/mission_planner.cpp)

set(FIRMWARE_SRCS
        src/qrotor_flight.cpp
        src/state_machine.cpp
        src/command_manager.cpp
        src/navio_board.cpp
        src/mixer.cpp
        src/mixer2.cpp
        src/sensors.cpp
        src/param_server.cpp)


list(APPEND FIRMWARE_SRCS ${PLANNING_SRCS} ${CONTROL_SRCS} ${ESTIMATION_SRCS} ${EIGEN_ESTIMATION_SRCS} ${MATLAB_ESTIMATION_SRCS} ${PERIPHERALS_SRCS})
add_library(qrotor_firmware ${FIRMWARE_SRCS})
target_include_directories(qrotor_firmware PUBLIC . include)
target_link_libraries(qrotor_firmware navio realsense2 mathlib  matlab_codegen  ${catkin_LIBRARIES})
add_dependencies(qrotor_firmware navio realsense2 mathlib  matlab_codegen  ${catkin_EXPORTED_TARGETS})

#////////////////////////////////////////////////////////
#/////////////////////// executables ////////////////////
#////////////////////////////////////////////////////////
add_executable(firmware_exe src/main.cpp)
add_dependencies(firmware_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(firmware_exe navio qrotor_firmware ${catkin_LIBRARIES})
# target_include_directories()

add_library(firmware_params_reconfig src/nodes/firmware_params_reconfig.cpp src/nodes/firmware_io.cpp)
add_dependencies(firmware_params_reconfig ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(firmware_params_reconfig qrotor_firmware ${catkin_LIBRARIES})

set(FIRMWARE_NODE_SRCS
        src/nodes/firmware_node.cpp
        src/nodes/firmware_io.cpp
        src/nodes/firmware_interface.cpp
        src/nodes/firmware_params_reconfig.cpp)
add_executable(firmware_node ${FIRMWARE_NODE_SRCS})
add_dependencies(firmware_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(firmware_node navio qrotor_firmware firmware_params_reconfig ${catkin_LIBRARIES})
# target_include_directories()

add_executable(pose_estimation_node src/nodes/pose_estimation_node.cpp)
add_dependencies(pose_estimation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_estimation_node navio qrotor_firmware ${catkin_LIBRARIES})


add_executable(tracking_camera_node src/nodes/tracking_camera_node.cpp)
add_dependencies(tracking_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_camera_node qrotor_firmware ${catkin_LIBRARIES})

add_executable(motor_calibration_node src/nodes/motor_calibration_node.cpp)
add_dependencies(motor_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_calibration_node ${catkin_LIBRARIES} qrotor_firmware)

############################
### test scripts (bad code)
############################
add_executable(dyn_reconfig_node src/tests/ddynreconfig.cpp)
add_dependencies(dyn_reconfig_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dyn_reconfig_node ${catkin_LIBRARIES})

add_executable(parallel_threads src/tests/parallel_threads.cpp)
add_dependencies(parallel_threads ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parallel_threads ${catkin_LIBRARIES})

add_executable(test_qcqp_fmincon src/tests/test_qcqp_fmincon.cpp)
add_dependencies(test_qcqp_fmincon ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_qcqp_fmincon matlab_codegen ${catkin_LIBRARIES})

add_executable(hello_world src/tests/hello_world.cpp)
add_dependencies(hello_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hello_world ${catkin_LIBRARIES})
