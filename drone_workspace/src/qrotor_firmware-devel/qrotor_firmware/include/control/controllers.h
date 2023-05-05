#ifndef QROTOR_FIRMWARE_CONTROLLERS_H
#define QROTOR_FIRMWARE_CONTROLLERS_H

///
/// attitude control
///
#include "control/attitude_controller.h"
#include "control/euler_angle_ppid.h"
#include "control/euler_pid.h"
#include "control/mueller_controller.h"
// #include "control/attitude_geometric_clf_qp.h"
// #include "control/attitude_vbl_lqr.h"
#include "control/attitude_control_px4.h"
#include "control/attitude_geometric_control.h"

///
/// position control
///
#include "control/position_controller.h"
#include "control/position_pid.h"
// #include "control/position_clf_qp.h"
// #include "control/position_mpc.h"

#endif // QROTOR_FIRMWARE_CONTROLLERS_H
