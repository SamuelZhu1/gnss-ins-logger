// WGS84 conversions and a local NED frame. Always double precision: these run
// at the GNSS rate (10 Hz), so the cost of software doubles on the ESP32 is small.
#pragma once

#include <Eigen/Core>

namespace gins {

struct Lla {
    double lat_deg = 0;
    double lon_deg = 0;
    double h_m = 0;  // ellipsoidal height
};

Eigen::Vector3d lla_to_ecef(const Lla& lla);
Lla ecef_to_lla(const Eigen::Vector3d& ecef);

// Normal gravity (Somigliana with a free-air height term), m/s^2.
double normal_gravity(double lat_deg, double h_m);

// Local tangent frame: north, east, down from a fixed origin.
class LocalFrame {
public:
    LocalFrame() = default;
    explicit LocalFrame(const Lla& origin);

    bool valid() const { return valid_; }
    const Lla& origin() const { return origin_; }
    Eigen::Vector3d to_ned(const Lla& lla) const;
    Lla to_lla(const Eigen::Vector3d& ned) const;

private:
    bool valid_ = false;
    Lla origin_;
    Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r_ne_ = Eigen::Matrix3d::Identity();  // ECEF to NED
};

}  // namespace gins
