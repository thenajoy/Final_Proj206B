#ifndef __QROTOR_FIRMWARE_RS_T265_H__
#define __QROTOR_FIRMWARE_RS_T265_H__
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <pthread.h>

#include "external_pose_handler.hpp"
#include "data_structures.h"
#include "utils.h"

namespace qrotor_firmware {

class RSt265Handler : public ExternalPoseHandler {
private:
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;

  //  framesets
  rs2::frameset frames;
  // frames
  rs2::frame f;

  // rs2 pose data
  rs2_pose pose_data{};
  unsigned long pose_data_time{};

  // lock when writing data to pose_data
  pthread_mutex_t plock{};

  unsigned long prev_time_{}, curr_time_{};

public:
  RSt265Handler();
  ~RSt265Handler();

  int init() override ;
  int run() override ;
  void transform_pose() override;
  //    PoseWithCovariance _pose;
};

} // namespace qrotor_firmware

#endif // __QROTOR_FIRMWARE_RS_T265_H__
