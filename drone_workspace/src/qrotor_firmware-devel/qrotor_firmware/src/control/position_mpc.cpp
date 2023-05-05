//
// Created by kotaru on 5/23/21.
//
#include "control/position_mpc.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

PositionMPC::PositionMPC(FlightController &_flightController)
    : PositionController(_flightController) {

}

PositionMPC::~PositionMPC() = default;

void PositionMPC::init() {
  float dt = 1 / firmware_.loop_rate();

  // dynamics
  nlc::MatrixX<float> A, B;
  A.resize(6, 6);
  B.resize(6, 3);
  A.setIdentity();
  A.topRightCorner(3, 3) += dt * Eigen::Matrix<float, 3, 3>::Identity();
  B << 0.5 * dt * dt * Eigen::Matrix<float, 3, 3>::Identity(),
      dt * Eigen::Matrix<float, 3, 3>::Identity();
  std::cout << "Initializing dynamics ... " << std::endl;
  std::cout << "A: \n" << A << std::endl;
  std::cout << "B: \n" << B << std::endl;
  pos_mpc_.init_dynamics(A, B);

  // gains
  nlc::MatrixX<float> Q, P, R;
  Q.resize(6, 6);
  P.resize(6, 6);
  R.resize(3, 3);
  Q << 1000 * Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero(),
      Eigen::Matrix3f::Zero(), 100 * Eigen::Matrix3f::Identity();
  P << 8.1314, 0.0000, -0.0000, 0.6327, -0.0000, -0.0000, 0.0000, 8.1314,
      0.0000, 0.0000, 0.6327, 0.0000, -0.0000, 0.0000, 8.1314, -0.0000, 0.0000,
      0.6327, 0.6327, 0.0000, -0.0000, 0.2606, -0.0000, -0.0000, -0.0000,
      0.6327, 0.0000, -0.0000, 0.2606, 0.0000, -0.0000, 0.0000, 0.6327, -0.0000,
      0.0000, 0.2606;
  P = 1e4 * P;
  R = 1 * Eigen::Matrix<float, 3, 3>::Identity();
  pos_mpc_.set_mpc_gains(Q, P, R);

  // state & input bounds
//  auto u_min = THRUST_VECTOR_LB / firmware_.vehicle_params_.mass_ - E3 * G_SI_F;
//  auto u_max = THRUST_VECTOR_UB / firmware_.vehicle_params_.mass_ - E3 * G_SI_F;
  Eigen::Matrix<float, 6, 1> state_lb, state_ub;
  Eigen::Matrix<float, 3, 1> input_lb, input_ub;
  state_lb << -2, -2.5, 0, -5, -5, -5;
  state_ub << 2, 2.5, 4, 5, 5, 5;
  input_lb = -G_SI_F * Eigen::Matrix<float, 3, 1>::Ones();
  input_ub = -input_lb;
  pos_mpc_.set_input_bounds(input_lb, input_ub);
  pos_mpc_.set_state_bounds(state_lb, state_ub);

  // construct the mpc
  pos_mpc_.construct();

  uOpt.resize(3, 1);
  zOpt.resize(30, 1);

  Logger::STATUS("Position MPC initialized!");
}

void PositionMPC::run(float dt) {
  state << firmware_.pos_estimator_->pose().position(0),
      firmware_.pos_estimator_->pose().position(1),
      firmware_.pos_estimator_->pose().position(2),
      firmware_.pos_estimator_->pose().velocity(0),
      firmware_.pos_estimator_->pose().velocity(1),
      firmware_.pos_estimator_->pose().velocity(2);
  goal_state << firmware_.mission_planner_->pose_des().position(0),
      firmware_.mission_planner_->pose_des().position(1),
      firmware_.mission_planner_->pose_des().position(2),
      firmware_.mission_planner_->pose_des().velocity(0),
      firmware_.mission_planner_->pose_des().velocity(1),
      firmware_.mission_planner_->pose_des().velocity(2);

//  this->debug();

  // running mpc controller
  zOpt = pos_mpc_.run((state - goal_state));
  uOpt = zOpt.block(0, 0, 3, 1);

  matrix::Vector3f u{uOpt(0), uOpt(1), uOpt(2)};
  u += (firmware_.mission_planner_->pose_des().acceleration + E3 * G_SI) *
      firmware_.vehicle_params_.mass_;

//  std::cout << " thrust_v " << u(0) << " " << u(1) << " " << u(2) << "\n" << std::endl;

  // send thrust vector to attitude controller
  firmware_.att_controller_->set_thrust_vector(u);

}

void PositionMPC::init_mpc() {}

void PositionMPC::run_mpc() {}

}

