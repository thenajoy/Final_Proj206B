//
// Created by kotaru on 6/3/21.
// Source: https://github.com/PX4/PX4-Autopilot/blob/f9d8c613b048f58eb3110e9af13cb3a89c4c866f/src/modules/mc_pos_control/PositionControl/ControlMath.hpp
//
#ifndef QROTOR_FIRMWARE_CONTROL_MATH_HPP
#define QROTOR_FIRMWARE_CONTROL_MATH_HPP

#include "Matrix/matrix/math.hpp"
#include <cmath>
#include <algorithm>

namespace control_math {
/**
 * Converts thrust vector and yaw set-point to a desired attitude.
 * @param thr_sp desired 3D thrust vector
 * @param yaw_sp the desired yaw
 * @param att_sp attitude setpoint to fill
 */
//void thrustToAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

/**
 * Limits the tilt angle between two unit vectors
 * @param body_unit unit vector that will get adjusted if angle is too big
 * @param world_unit fixed vector to measure the angle against
 * @param max_angle maximum tilt angle between vectors in radians
 */
void limitTilt(matrix::Vector3f &body_unit, const matrix::Vector3f &world_unit, const float max_angle);

/**
 * Converts a body z vector and yaw set-point to a desired attitude.
 * @param body_z a world frame 3D vector in direction of the desired body z axis
 * @param yaw_sp the desired yaw setpoint
 * @param att_sp attitude setpoint to fill
 */
//void bodyzToAttitude(matrix::Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

/**
 * Outputs the sum of two vectors but respecting the limits and priority.
 * The sum of two vectors are constraint such that v0 has priority over v1.
 * This means that if the length of (v0+v1) exceeds max, then it is constraint such
 * that v0 has priority.
 *
 * @param v0 a 2D vector that has priority given the maximum available magnitude.
 * @param v1 a 2D vector that less priority given the maximum available magnitude.
 * @return 2D vector
 */
matrix::Vector2f constrainXY(const matrix::Vector2f &v0, const matrix::Vector2f &v1, const float &max);

/**
 * This method was used for smoothing the corners along two lines.
 *
 * @param sphere_c
 * @param sphere_r
 * @param line_a
 * @param line_b
 * @param res
 * return boolean
 *
 * Note: this method is not used anywhere and first requires review before usage.
 */
bool cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r, const matrix::Vector3f &line_a,
                       const matrix::Vector3f &line_b, matrix::Vector3f &res);

/**
 * Adds e.g. feed-forward to the setpoint making sure existing or added NANs have no influence on control.
 * This function is udeful to support all the different setpoint combinations of position, velocity, acceleration with NAN representing an uncommited value.
 * @param setpoint existing possibly NAN setpoint to add to
 * @param addition value/NAN to add to the setpoint
 */
void addIfNotNan(float &setpoint, const float addition);

/**
 * _addIfNotNan for Vector3f treating each element individually
 * @see _addIfNotNan
 */
void addIfNotNanVector3f(matrix::Vector3f &setpoint, const matrix::Vector3f &addition);

/**
 * Overwrites elements of a Vector3f which are NaN with zero
 * @param vector possibly containing NAN elements
 */
void setZeroIfNanVector3f(matrix::Vector3f &vector);
}

#endif //QROTOR_FIRMWARE_CONTROL_MATH_HPP
