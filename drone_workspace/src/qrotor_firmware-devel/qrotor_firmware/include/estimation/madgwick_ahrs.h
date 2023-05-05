//
// Created by kotaru on 3/16/21.
//
//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

#ifndef __QROTOR_FIRMWARE_MADGWICK_AHRS_H__
#define __QROTOR_FIRMWARE_MADGWICK_AHRS_H__

#include "estimation/attitude_estimator.h"

namespace qrotor_firmware {
class FlightController;

class MadgwickAHRS : public AttitudeEstimator {

protected:
  float beta = 0.1; // 2 * proportional gain (Kp)

  void update(float dt);

public:
  explicit MadgwickAHRS(FlightController &_flightcontroller);
  ~MadgwickAHRS();

  virtual void init() override;
  virtual void run(float dt) override;
};
}
#endif //__QROTOR_FIRMWARE_MADGWICK_AHRS_H__
