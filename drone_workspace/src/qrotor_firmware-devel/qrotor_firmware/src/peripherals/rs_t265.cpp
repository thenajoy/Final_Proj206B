#include <log.hpp>
#include "peripherals/rs_t265.h"
namespace qrotor_firmware {

RSt265Handler::RSt265Handler() : ExternalPoseHandler() {
  //  pthread_mutex_lock(&plock);
  pose_data.translation.x = 0;
  pose_data.translation.y = 0;
  pose_data.translation.z = 0;
  pose_data.velocity.x = 0;
  pose_data.velocity.y = 0;
  pose_data.velocity.z = 0;
  pose_data.angular_velocity.x = 0;
  pose_data.angular_velocity.y = 0;
  pose_data.angular_velocity.z = 0;
  pose_data.acceleration.x = 0;
  pose_data.acceleration.y = 0;
  pose_data.acceleration.z = 0;
  pose_data.rotation.x = 0;
  pose_data.rotation.y = 0;
  pose_data.rotation.z = 0;
  pose_data.rotation.w = 1;
  //  pthread_mutex_unlock(&plock);
}

RSt265Handler::~RSt265Handler() = default;

int RSt265Handler::init() {
  try {
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);
    return EXIT_SUCCESS;
  }
  catch (const rs2::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
              << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  prev_time_ = utils::get_current_time();
}

int RSt265Handler::run() {
  curr_time_ = utils::get_current_time();
  //  Logger::STATUS(utils::Cat("rs-loop-dt_s: ", float(curr_time_-prev_time_)/1000000.0));
  prev_time_ = curr_time_;

  // Main loop

  // Wait for the next set of frames from the camera
  frames = pipe.wait_for_frames();
  // Get a frame from the pose stream
  f = frames.first_or_default(RS2_STREAM_POSE);
  // Cast the frame to pose_frame and get its data
  pose_data = f.as<rs2::pose_frame>().get_pose_data();
  new_pose_obtained_ = true;
  //   // Todo why did they stop working suddenly
  //  Logger::STATUS(utils::Cat("position: ", pose_data.translation.x,
  //                            " ", pose_data.translation.x,
  //                            " ", pose_data.translation.z));
  //  Logger::STATUS(utils::Cat("rotation: ", pose_data.rotation.w,
  //                            " ", pose_data.rotation.x,
  //                            " ", pose_data.rotation.x,
  //                            " ", pose_data.rotation.z));
  //  pthread_mutex_lock(&plock);
  this->transform_pose();

  //  pthread_mutex_unlock(&plock);
  return true;
}

void RSt265Handler::transform_pose() {
  matrix::Vector3f pos, vel;
  pos = {pose_data.translation.x, pose_data.translation.y, pose_data.translation.z};
  vel = {pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z};
  matrix::Quatf quat = {pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z};
  //  Logger::STATUS(utils::Cat("position: ", pos(0), " ", pos(1), " ", pos(2)));
  if ((pos.norm() > 10) || (vel.norm() > 10) || std::isnan(quat.norm()))
    return;
  //  pose_.position = pos;
  //  pose_.velocity = vel;
  //  pose_.acceleration = {pose_data.acceleration.x, pose_data.acceleration.y, pose_data.acceleration.z};
  //  pose_.ang_vel = {pose_data.angular_velocity.x, pose_data.angular_velocity.y, pose_data.angular_velocity.z};
  //  pose_.quat = quat;
  //  pose_.timestamp_us = utils::get_current_time();

  // adjust for the tilt in the mount
  matrix::Dcmf rot;
  rot = quat.to_dcm() * (cam_quat.to_dcm()).transpose();
  matrix::Quatf q(rot);

  // convert to drone body-frame // TODO customize this according to the drone
  pose_.position = {pos(0), -pos(2), pos(1)};
  pose_.quat = {q(0), q(1), -q(3), q(2)};
  pose_.velocity = {vel(0), -vel(2), vel(1)};
  pose_.acceleration = {pose_data.acceleration.x, -pose_data.acceleration.z, pose_data.acceleration.y};
  pose_.ang_vel = {pose_data.angular_velocity.x, -pose_data.angular_velocity.z, pose_data.angular_velocity.y};
  pose_.timestamp_us = utils::get_current_time();
}

}
