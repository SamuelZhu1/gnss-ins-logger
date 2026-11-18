#include "gins/geo.hpp"

#include <cmath>

namespace gins {
namespace {
constexpr double kA = 6378137.0;              // WGS84 semi-major axis
constexpr double kF = 1.0 / 298.257223563;    // flattening
constexpr double kE2 = kF * (2.0 - kF);       // first eccentricity squared
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}  // namespace

Eigen::Vector3d lla_to_ecef(const Lla& lla) {
    const double lat = lla.lat_deg * kDegToRad;
    const double lon = lla.lon_deg * kDegToRad;
    const double s = std::sin(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * s * s);
    return {(n + lla.h_m) * std::cos(lat) * std::cos(lon),
            (n + lla.h_m) * std::cos(lat) * std::sin(lon),
            (n * (1.0 - kE2) + lla.h_m) * s};
}

Lla ecef_to_lla(const Eigen::Vector3d& e) {
    const double lon = std::atan2(e.y(), e.x());
    const double p = std::hypot(e.x(), e.y());
    double lat = std::atan2(e.z(), p * (1.0 - kE2));
    double h = 0;
    // Fixed-point iteration converges to well under a millimetre in a few steps.
    for (int i = 0; i < 6; ++i) {
        const double s = std::sin(lat);
        const double n = kA / std::sqrt(1.0 - kE2 * s * s);
        h = p / std::cos(lat) - n;
        lat = std::atan2(e.z(), p * (1.0 - kE2 * n / (n + h)));
    }
    return {lat / kDegToRad, lon / kDegToRad, h};
}

double normal_gravity(double lat_deg, double h_m) {
    const double s2 = std::pow(std::sin(lat_deg * kDegToRad), 2);
    const double g0 = 9.7803253359 * (1.0 + 0.00193185265241 * s2) / std::sqrt(1.0 - kE2 * s2);
    return g0 - 3.086e-6 * h_m;
}

LocalFrame::LocalFrame(const Lla& origin) : valid_(true), origin_(origin) {
    origin_ecef_ = lla_to_ecef(origin);
    const double lat = origin.lat_deg * kDegToRad;
    const double lon = origin.lon_deg * kDegToRad;
    const double sl = std::sin(lat), cl = std::cos(lat);
    const double so = std::sin(lon), co = std::cos(lon);
    r_ne_ << -sl * co, -sl * so, cl,
             -so, co, 0,
             -cl * co, -cl * so, -sl;
}

Eigen::Vector3d LocalFrame::to_ned(const Lla& lla) const {
    return r_ne_ * (lla_to_ecef(lla) - origin_ecef_);
}

Lla LocalFrame::to_lla(const Eigen::Vector3d& ned) const {
    return ecef_to_lla(origin_ecef_ + r_ne_.transpose() * ned);
}

}  // namespace gins
