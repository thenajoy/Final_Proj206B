#include "estimation/pose_iekf.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

PoseInvariantEKF::PoseInvariantEKF(FlightController &_flightcontroller)
    : PoseEstimator(_flightcontroller), g_(-E3 * G_SI) {}

PoseInvariantEKF::~PoseInvariantEKF() = default;

void PoseInvariantEKF::init() {
  Logger::ERROR("Setup the noise parameters!");
  invariant_ekf_.noise_.set_accel_std(2.56, 4.208, 2.794);
  invariant_ekf_.noise_.set_gyro_std(0.0424, 0.0397, 0.0952);
  invariant_ekf_.updateNoiseParams();
}

void PoseInvariantEKF::externalPoseWithCovariance_to_RigidbodyState() {
  //
  // convert PX4 MATRIX to EIGEN MATRIX and RigidbodyState
  //
  firmware_.ext_pose_handler_->reset_new_pose_obtained();

  Eigen::Matrix3f R;
  matrix::Dcmf m(firmware_.ext_pose_handler_->pose().quat);
  R << m(0, 0), m(0, 1), m(0, 2), m(1, 0), m(1, 1), m(1, 2), m(2, 0), m(2, 1),
      m(2, 2);
  Eigen::Vector3f v, x;
  v << firmware_.ext_pose_handler_->pose().velocity(0),
      firmware_.ext_pose_handler_->pose().velocity(1),
      firmware_.ext_pose_handler_->pose().velocity(2);
  x << firmware_.ext_pose_handler_->pose().position(0),
      firmware_.ext_pose_handler_->pose().position(1),
      firmware_.ext_pose_handler_->pose().position(2);
  pose_meas_.update(R, v, x);
}

void PoseInvariantEKF::estimatedRigidbodyState_to_PoseWithCovariance() {

  matrix::Dcmf rot;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rot(i, j) = invariant_ekf_.state().rotation()(i, j);
    }
  }
  auto rotm = invariant_ekf_.state().rotation().block(0, 0, 3, 3);
  float det_rotm = rotm.determinant();
  if (!(0.99 < det_rotm < 1.01)) {
    Logger::WARN(utils::Cat("estimated rotation determinant: ", det_rotm));
  }

  pose_.quat = matrix::Quatf(rot);
  pose_.velocity = {invariant_ekf_.state().velocity()(0),
                    invariant_ekf_.state().velocity()(1),
                    invariant_ekf_.state().velocity()(2)};
  pose_.position = {invariant_ekf_.state().position()(0),
                    invariant_ekf_.state().position()(1),
                    invariant_ekf_.state().position()(2)};
  //  pose_.print();
}

void PoseInvariantEKF::time_update(const float &_dt) {

  //  std::cout << "poseiekf run "<< std::endl;
  if (!iekf_initialized) {
    // TODO add initialization loop or state_machine_variable
    if (firmware_.ext_pose_handler_->new_pose_obtained()) {

      externalPoseWithCovariance_to_RigidbodyState();

      // temporary fix
      pose_meas_.set_velocity(Eigen::Vector3f::Zero());

      invariant_ekf_.init(pose_meas_);
      iekf_initialized = true;
      Logger::STATUS("Invariant EKF initialized!");
    }
    return;
  }

  w = {firmware_.sensors_.imu().gyro(0), firmware_.sensors_.imu().gyro(1),
       firmware_.sensors_.imu().gyro(2)};
  a = {firmware_.sensors_.imu().accel(0), firmware_.sensors_.imu().accel(1),
       firmware_.sensors_.imu().accel(2)};

  // apply low-pass filters
  //  w = gyro_low_pass_.apply(w);
  //  a = accel_low_pass_.apply(a);

  imu_meas_.update(0, a(0), a(1), a(2), w(0), w(1), w(2));

  auto start_time = utils::get_current_time();
  invariant_ekf_.timeUpdate(_dt, imu_meas_);
  auto end_time = utils::get_current_time();

  printf("time_update: dt_s: %f, ", float(end_time - start_time) / 1000000.0);
}

void PoseInvariantEKF::meas_update() {

  if (firmware_.ext_pose_handler_->new_pose_obtained()) {
    // if new pose measurement, perform measurement update
    firmware_.ext_pose_handler_->reset_new_pose_obtained();

    auto start_time = utils::get_current_time();
    externalPoseWithCovariance_to_RigidbodyState();
    invariant_ekf_.measUpdate(pose_meas_);
    auto end_time = utils::get_current_time();

    printf("meas_update: dt_s: %f", float(end_time - start_time) / 1000000.0);
  }
  //  printf("\n");
  //  estimatedRigidbodyState_to_PoseWithCovariance();
  std::cout << "----" << std::endl;
  std::cout << invariant_ekf_.state().X() << std::endl;
}

void PoseInvariantEKF::run(float dt) {
  time_update(dt);
  meas_update();
}

} // namespace qrotor_firmware
