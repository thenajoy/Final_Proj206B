//
// Created by kotaru on 4/27/21.
//

#ifndef QROTOR_FIRMWARE_CIRCULAR_TRAJECTORY_H_
#define QROTOR_FIRMWARE_CIRCULAR_TRAJECTORY_H_
#include "base_trajectory.hpp"

namespace qrotor_firmware {

class CircularTrajectory : public Trajectory {
protected:

  void circle_traj(float t) {
    float t10;
    float t11;
    float t12;
    float t13;
    float t14;
    float t15;
    float t16;
    float t17;
    float t6;
    float t7;
    float t8;
    float t9;
    //     This function was generated by the Symbolic Math Toolbox version 8.7.
    //     28-Apr-2021 03:23:33
    t6 = yaw_rate * yaw_rate;
    t7 = phi[0] + t * w[0];
    t8 = phi[1] + t * w[1];
    t9 = phi[2] + t * w[2];
    t10 = t * yaw_rate + yaw;
    t11 = std::cos(t7);
    t12 = std::cos(t8);
    t13 = std::cos(t9);
    t14 = std::cos(t10);
    t15 = std::sin(t7);
    t16 = std::sin(t8);
    t17 = std::sin(t9);
    t7 = std::sin(t10);
    t8 = r[0] * t11;
    t9 = r[1] * t16;
    t10 = r[2] * t17;
    flats_->p[0] = p0[0] + t8;
    flats_->p[1] = p0[1] + t9;
    flats_->p[2] = p0[2] + t10;
    flats_->v[0] = -r[0] * t15 * w[0];
    flats_->v[1] = r[1] * t12 * w[1];
    flats_->v[2] = r[2] * t13 * w[2];
    flats_->a[0] = -r[0] * t11 * (w[0] * w[0]);
    flats_->a[1] = -r[1] * t16 * (w[1] * w[1]);
    flats_->a[2] = -r[2] * t17 * (w[2] * w[2]);
    flats_->da[0] = r[0] * t15 * std::pow(w[0], 3.0F);
    flats_->da[1] = -r[1] * t12 * std::pow(w[1], 3.0F);
    flats_->da[2] = -r[2] * t13 * std::pow(w[2], 3.0F);
    flats_->d2a[0] = t8 * std::pow(w[0], 4.0F);
    flats_->d2a[1] = t9 * std::pow(w[1], 4.0F);
    flats_->d2a[2] = t10 * std::pow(w[2], 4.0F);
    flats_->b1[0] = t14;
    flats_->b1[1] = t7;
    flats_->b1[2] = 0.0F;
    flats_->db1[0] = -t7 * yaw_rate;
    flats_->db1[1] = t14 * yaw_rate;
    flats_->db1[2] = 0.0F;
    flats_->d2b1[0] = -t6 * t14;
    flats_->d2b1[1] = -t6 * t7;
    flats_->d2b1[2] = 0.0F;
  }

public:
  CircularTrajectory() : Trajectory() {}
  ~CircularTrajectory() = default;

  void run(const float &t) override { circle_traj(t); }
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_CIRCULAR_TRAJECTORY_H_
