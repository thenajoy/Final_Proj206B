/*
 * @file common.h
 * @author vkotaru
 * @date 12/17/20
 */
#ifndef QROTOR_GAZEBO_COMMON_H_
#define QROTOR_GAZEBO_COMMON_H_

#include <eigen3/Eigen/Dense>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>

#include <qrotor_gazebo/gz_compat.h>

namespace qrotor_gazebo {

inline Eigen::Vector3d vec3_to_eigen_from_gazebo(GazeboVector vec) {
  Eigen::Vector3d out;
  out << GZ_COMPAT_GET_X(vec), GZ_COMPAT_GET_Y(vec), GZ_COMPAT_GET_Z(vec);
  return out;
}

inline GazeboVector vec3_to_gazebo_from_eigen(Eigen::Vector3d vec) {
  GazeboVector out(vec(0), vec(1), vec(2));
  return out;
}

inline Eigen::Matrix3d rotation_to_eigen_from_gazebo(GazeboQuaternion quat) {
  Eigen::Quaterniond eig_quat(GZ_COMPAT_GET_W(quat), GZ_COMPAT_GET_X(quat),
                              GZ_COMPAT_GET_Y(quat), GZ_COMPAT_GET_Z(quat));
  return eig_quat.toRotationMatrix();
}

struct Pose3d {
public:
  Pose3d() {}
  Pose3d(const gazebo::physics::LinkPtr link_) {
    gpose = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
    gvel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
    gomega = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

    if ((abs(gpose.Pos().X()) > 100) || (abs(gpose.Pos().Y()) > 100) ||
        (abs(gpose.Pos().Z()) > 100)) {
      gzerr << "bad pose reading" << std::endl;
    }

    // state in NWU frame
    pos = vec3_to_eigen_from_gazebo(GZ_COMPAT_GET_POS(gpose));
    rot = rotation_to_eigen_from_gazebo(GZ_COMPAT_GET_ROT(gpose));
    vel = vec3_to_eigen_from_gazebo(gvel);
    omega = vec3_to_eigen_from_gazebo(gomega);
  }
  GazeboPose gpose;
  GazeboVector gvel;
  GazeboVector gomega;

  Eigen::Vector3d pos; // Position of MAV in NWU wrt initial position
  Eigen::Matrix3d rot; // Rotation of MAV in NWU wrt initial position
  Eigen::Vector3d vel; // Body-fixed velocity of MAV wrt initial position (NWU)
  Eigen::Vector3d omega; // Body-fixed angular velocity of MAV (NWU)
  double t;              // current time
};

} // namespace qrotor_gazebo

#endif // QROTOR_GAZEBO_COMMON_H_
