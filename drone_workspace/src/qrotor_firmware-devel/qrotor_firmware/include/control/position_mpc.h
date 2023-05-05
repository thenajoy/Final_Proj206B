//
// Created by kotaru on 5/23/21.
//
#ifndef QROTOR_FIRMWARE_POSITION_MPC_H_
#define QROTOR_FIRMWARE_POSITION_MPC_H_

#include <queue>
#include "control/position_controller.h"
#include "controls/linear_mpc.h"

namespace qrotor_firmware {
namespace nlc = nonlinear_controls;

class FlightController;

class PositionMPC : public PositionController {
protected:
  matrix::Vector3f pos_err{}, vel_err{};
  float dt_s{1 / 500.0};
  int N{10};
  std::queue<PoseWithCovariance> xd{};

  bool is_mpc_initialized{false};
  nlc::LinearMPC<float> pos_mpc_{10, 6, 3};
  nlc::MatrixX<float>zOpt, uOpt;
  Eigen::Matrix<float, 6, 1> goal_state, state;

public:
  explicit PositionMPC(FlightController &_flightController);
  ~PositionMPC();

  void init() override;
  void run(float dt) override;

  void init_mpc();
  void run_mpc();
};

}

#endif //QROTOR_FIRMWARE_POSITION_MPC_H_
