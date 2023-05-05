#ifndef QROTOR_FIRMWARE_LIE_ALGEBRA_H
#define QROTOR_FIRMWARE_LIE_ALGEBRA_H

#include "Matrix/matrix/math.hpp"
namespace lie_algebra {

static const double TOLERANCE = 1e-10;

inline matrix::Matrix3f hat(const matrix::Vector3f& v) {
    // hatmap: (also known as cross-map or skew-map)
    float data[9] = {0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0};
    matrix::Matrix3f M(data);
    return M;
}

inline matrix::Matrix3f expSO3(const matrix::Vector3f& w) {
    // exponential map for SO(3)
    matrix::Matrix3f A = hat(w);
    matrix::Matrix3f R;
    R.identity();
    double theta = w.norm();
    if (theta > TOLERANCE) {
        R += A * float(sin(theta) / theta) + (A * A) * float((1 - cos(theta)) / (theta * theta));
    }
    return R;
}


}

#endif // QROTOR_FIRMWARE_LIE_ALGEBRA_H
