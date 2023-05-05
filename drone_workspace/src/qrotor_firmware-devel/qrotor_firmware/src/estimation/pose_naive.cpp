#include "estimation/pose_naive.h"
#include "qrotor_flight.h"
namespace qrotor_firmware {

PoseEstNaive::PoseEstNaive(FlightController &_flightcontroller) : PoseEstimator(_flightcontroller) {

  _lpf_velocity.set_cutoff_frequency(firmware_.loop_rate(), 50.0f); // cut_off freq
}

PoseEstNaive::~PoseEstNaive() = default;

void PoseEstNaive::init() {
  pose_.reset();
  Logger::STATUS(std::string("PoseEstNaive initialized!"));
  std::cout << "Position: " << std::setprecision(3) << std::fixed << pose_.position(0) << " " <<
            pose_.position(1) << " " << pose_.position(2) << " (meters)" << std::endl;
  std::cout << "Velocity: " << std::setprecision(3) << std::fixed << pose_.velocity(0) << " " <<
            pose_.velocity(1) << " " << pose_.velocity(2) << " (meters)" << std::endl;
}

void PoseEstNaive::compute_tracking_camera_init_offset() {
  // Todo add a for loop to compute the average
  std::cout << "compute_tracking_camera_init_offset" << std::endl;
  tcam_init_offset_ = firmware_.ext_pose_handler_->mocap_pose();
  std::cout << "initial position: " << tcam_init_offset_.position(0) << " "
            << tcam_init_offset_.position(1) << " "
            << tcam_init_offset_.position(2) << std::endl;

  matrix::Quatf qt = firmware_.ext_pose_handler_->pose().quat;
  matrix::Quatf qm = firmware_.ext_pose_handler_->mocap_pose().quat;
  mocap_tcam_quat_offset = matrix::Quatf((qt.to_dcm().T() * qm.to_dcm()));
  is_initialized = true;
}

void PoseEstNaive::run(float dt) {
  //  PoseWithCovariance tpose_;
  //  firmware_.ext_pose_handler_->get_pose_info(pose_);
  if (firmware_.params_->get(Params::EXTERNAL_POSE) == 0) {
    // using pose from tracking camera directly
    pose_ = firmware_.ext_pose_handler_->pose();

  } else if (firmware_.params_->get(Params::EXTERNAL_POSE) == 1) {
    // using pose from motion capture directly
    pose_ = firmware_.ext_pose_handler_->mocap_pose();

  } else if (firmware_.params_->get(Params::EXTERNAL_POSE) == 2) {
    if (!is_initialized) {
      if (firmware_.ext_pose_handler_->new_mocap_pose() && firmware_.ext_pose_handler_->new_pose_obtained()) {
        compute_tracking_camera_init_offset();
      }
      return;
    }


    // fusing tracking camera and motion capture pose using user-defined covariances
    float kp = firmware_.params_->get(Params::EXTERNAL_POSE_TCAM_COVAR);
    float km = firmware_.params_->get(Params::EXTERNAL_POSE_MOCAP_COVAR);
    matrix::Matrix3f K;
    K.setIdentity();
    K = kp / (kp + km) * K;

    PoseWithCovariance mpose_ = firmware_.ext_pose_handler_->mocap_pose();
    PoseWithCovariance tpose_ = firmware_.ext_pose_handler_->pose();
    //    std::cout << "tquat-before\t " << tpose_.quat(0) << " " << tpose_.quat(1) << " " << tpose_.quat(2) << " "
    //              << tpose_.quat(3)
    //              << "\n";
    //    std::cout << "tpose-before\t " << tpose_.position(0) << " " << tpose_.position(1) << " " << tpose_.position(2)
    //              << "\n";

    // adjust for the tracking-camera initial offset
    tpose_.position =
        tcam_init_offset_.quat.to_dcm() * firmware_.ext_pose_handler_->pose().position + tcam_init_offset_.position;
    tpose_.velocity = tcam_init_offset_.quat.to_dcm() * firmware_.ext_pose_handler_->pose().velocity;
    matrix::Quatf qt = firmware_.ext_pose_handler_->pose().quat;
    tpose_.quat = matrix::Quatf(qt.to_dcm() * mocap_tcam_quat_offset.to_dcm());
    //    qt.to_dcm().print();

    // position fusion
    matrix::Vector3f z = mpose_.position - tpose_.position;
    pose_ = tpose_;
    pose_.position = tpose_.position + K * z;

    //    std::cout << "tquat-after\t " << tpose_.quat(0) << " " << tpose_.quat(1) << " " << tpose_.quat(2) << " "
    //              << tpose_.quat(3)
    //              << "\n";
    //    std::cout << "mquat\t " << mpose_.quat(0) << " " << mpose_.quat(1) << " " << mpose_.quat(2) << " "
    //              << mpose_.quat(3)
    //              << "\n";
    //
    //    std::cout << "tpose-after\t " << tpose_.position(0) << " " << tpose_.position(1) << " " << tpose_.position(2)
    //              << "\n";
    //    std::cout << "mpose\t " << mpose_.position(0) << " " << mpose_.position(1) << " " << mpose_.position(2) << "\n";
    //    std::cout << "pose \t " << pose_.position(0) << " " << pose_.position(1) << " " << pose_.position(2) << "\n";

  } else {
    Logger::ERROR("Undefined EXTERNAL_POSE!");
    // TODO define & set an error flag to the state-machine
  }
  pose_.timestamp_us = firmware_.get_firmware_time();

}

}
