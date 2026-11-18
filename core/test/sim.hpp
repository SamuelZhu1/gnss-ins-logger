// Synthetic bike rides for the tests: exact kinematics with lean in turns,
// then IMU noise and bias and GNSS noise on top. Never used as ride results.
#pragma once

#include <cstdint>
#include <vector>

#include "gins/geo.hpp"
#include "gins/types.hpp"

namespace sim {

struct Config {
    double static_s = 22;     // held still at the start (alignment)
    double ride_s = 120;      // riding time after the start
    double speed = 6;         // m/s cruise
    double radius = 40;       // m, turn radius
    bool figure_eight = false;
    double imu_hz = 200;
    double gnss_hz = 10;
    double accel_bias = 0.04;     // m/s^2, constant per axis (signs vary)
    double gyro_bias = 0.004;     // rad/s
    double accel_noise = 0.01;    // m/s^2/sqrt(Hz), sensor noise
    double vibration = 0.4;       // m/s^2 per sample, road vibration while moving
    double gyro_noise = 0.001;    // rad/s/sqrt(Hz)
    double gnss_pos_sigma = 0.5;  // m, white
    double gnss_vel_sigma = 0.05; // m/s
    bool noise = true;
    uint32_t seed = 1;
    gins::Lla origin{49.2606, -123.2460, 80.0};
};

struct Ride {
    std::vector<gins::ImuSample> imu;
    std::vector<gins::GnssFix> fixes;
    std::vector<int64_t> truth_t;         // at each fix time
    std::vector<gins::Lla> truth_lla;
    std::vector<Eigen::Vector3d> truth_ned;
    gins::LocalFrame frame;
};

Ride make_ride(const Config& c);

}  // namespace sim
