// Shared types for the GNSS/INS filter. Built for the host (double) and the
// ESP32-S3 (float, GINS_USE_FLOAT) from the same source.
#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gins {

#ifdef GINS_USE_FLOAT
using real = float;
#else
using real = double;
#endif

using Vec3 = Eigen::Matrix<real, 3, 1>;
using Mat3 = Eigen::Matrix<real, 3, 3>;
using Quat = Eigen::Quaternion<real>;

// Error state: dp(3) dv(3) dtheta(3) dba(3) dbg(3)
constexpr int kErr = 15;
constexpr int kP = 0;
constexpr int kV = 3;
constexpr int kTh = 6;
constexpr int kBa = 9;
constexpr int kBg = 12;
using ErrVec = Eigen::Matrix<real, kErr, 1>;
using ErrMat = Eigen::Matrix<real, kErr, kErr>;

// One IMU sample in the body frame (x forward, y right, z down).
// accel is specific force in m/s^2, gyro is angular rate in rad/s.
// The sample is treated as the average over the interval ending at t_us.
struct ImuSample {
    int64_t t_us = 0;  // GPS time, microseconds
    Vec3 accel = Vec3::Zero();
    Vec3 gyro = Vec3::Zero();
};

// One GNSS solution at its epoch time (not its arrival time).
struct GnssFix {
    int64_t t_us = 0;  // GPS time of the epoch, microseconds
    double lat_deg = 0;
    double lon_deg = 0;
    double h_m = 0;    // height above the WGS84 ellipsoid
    Vec3 vel_ned = Vec3::Zero();
    real h_acc = 99;   // 1 sigma, m
    real v_acc = 99;   // 1 sigma, m
    real s_acc = 9;    // 1 sigma, m/s
    uint8_t fix_type = 0;   // u-blox: 3 = 3D, 4 = GNSS + dead reckoning
    uint8_t carr_soln = 0;  // 0 none, 1 RTK float, 2 RTK fixed
    uint8_t num_sv = 0;
};

enum class Mode : uint8_t {
    kWaitFix = 0,   // no usable fix yet, local frame not set
    kAligning = 1,  // static alignment: roll, pitch, gyro bias
    kWaitYaw = 2,   // waiting for steady motion to take yaw from GNSS course
    kNav = 3,       // filter running with GNSS
    kDegraded = 4,  // filter running, no accepted GNSS for a long time
};

const char* mode_name(Mode m);

// Nominal state. Position is NED in metres from the local origin.
struct State {
    int64_t t_us = 0;
    Vec3 p = Vec3::Zero();
    Vec3 v = Vec3::Zero();
    Quat q = Quat::Identity();  // body to NED
    Vec3 ba = Vec3::Zero();
    Vec3 bg = Vec3::Zero();
};

struct FilterConfig {
    // White noise densities. The ICM-42688-P datasheet gives about 7e-4 m/s^2/rtHz
    // and 5e-5 rad/s/rtHz; these are inflated for frame vibration on a bike.
    real accel_noise = static_cast<real>(0.05);    // m/s^2/sqrt(Hz)
    real gyro_noise = static_cast<real>(2e-3);     // rad/s/sqrt(Hz)
    real accel_bias_rw = static_cast<real>(5e-4);  // m/s^3/sqrt(Hz)
    real gyro_bias_rw = static_cast<real>(2e-5);   // rad/s^2/sqrt(Hz)

    real gnss_gate = static_cast<real>(22.46);  // chi-square, 6 DOF, 99.9 %
    real gnss_r_scale = 1;                      // multiplies the receiver's own accuracy

    bool use_nhc = true;                        // bike motion constraint
    real nhc_sigma = static_cast<real>(0.3);    // m/s, lateral and vertical body velocity
    real nhc_min_speed = static_cast<real>(1.0);

    bool use_zupt = true;
    real zupt_sigma = static_cast<real>(0.02);  // m/s
    real zaru_sigma = static_cast<real>(0.005); // rad/s

    real max_dt = static_cast<real>(0.05);      // larger IMU gaps are clamped and counted
};

struct FilterDiag {
    uint32_t gnss_accepted = 0;
    uint32_t gnss_rejected = 0;
    uint32_t gnss_forced = 0;
    uint32_t nhc_updates = 0;
    uint32_t zupt_updates = 0;
    uint32_t imu_gaps = 0;
    real last_gnss_nis = 0;
};

}  // namespace gins
