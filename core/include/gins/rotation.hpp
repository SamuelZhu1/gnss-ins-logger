// Small rotation helpers: skew matrix, rotation vector to quaternion, and
// roll/pitch/yaw (ZYX, NED) conversions.
#pragma once

#include <cmath>

#include "gins/types.hpp"

namespace gins {

inline Mat3 skew(const Vec3& v) {
    Mat3 m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

// Quaternion for a rotation vector (axis * angle, rad).
inline Quat quat_from_rotvec(const Vec3& rv) {
    const real angle = rv.norm();
    if (angle < static_cast<real>(1e-8)) {
        // Second order small-angle form keeps the quaternion close to unit length.
        Quat q(1, rv.x() / 2, rv.y() / 2, rv.z() / 2);
        q.normalize();
        return q;
    }
    const real half = angle / 2;
    const real s = std::sin(half) / angle;
    return Quat(std::cos(half), rv.x() * s, rv.y() * s, rv.z() * s);
}

// Body to NED quaternion from roll, pitch, yaw in rad (ZYX order).
inline Quat quat_from_rpy(real roll, real pitch, real yaw) {
    const Quat qz(Eigen::AngleAxis<real>(yaw, Vec3::UnitZ()));
    const Quat qy(Eigen::AngleAxis<real>(pitch, Vec3::UnitY()));
    const Quat qx(Eigen::AngleAxis<real>(roll, Vec3::UnitX()));
    return (qz * qy * qx).normalized();
}

// Roll, pitch, yaw in rad from a body to NED quaternion.
inline Vec3 rpy_from_quat(const Quat& q) {
    const Mat3 r = q.toRotationMatrix();
    const real pitch = std::asin(std::max(static_cast<real>(-1), std::min(static_cast<real>(1), -r(2, 0))));
    const real roll = std::atan2(r(2, 1), r(2, 2));
    const real yaw = std::atan2(r(1, 0), r(0, 0));
    return Vec3(roll, pitch, yaw);
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

inline real wrap_pi(real a) {
    const real pi = static_cast<real>(kPi);
    while (a > pi) a -= 2 * pi;
    while (a < -pi) a += 2 * pi;
    return a;
}

}  // namespace gins
