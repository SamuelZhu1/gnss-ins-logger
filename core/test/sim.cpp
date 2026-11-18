#include "sim.hpp"

#include <cmath>
#include <random>

#include "gins/rotation.hpp"

namespace sim {
namespace {

constexpr double kG = 9.81;
constexpr int64_t kT0 = 100'000'000'000LL;  // arbitrary GPS time origin, us

struct Profile {
    const Config& c;
    // Speed: still, then a 5 s ramp, then cruise.
    double speed(double t) const {
        const double tm = t - c.static_s;
        if (tm <= 0) return 0;
        if (tm < 5) return c.speed * (0.5 - 0.5 * std::cos(kPiD * tm / 5));
        return c.speed;
    }
    // Yaw rate: straight for 10 s of riding, then turning (smoothly blended).
    double yaw_rate(double t) const {
        const double tm = t - c.static_s - 10;
        if (tm <= 0) return 0;
        const double blend = tm < 2 ? 0.5 - 0.5 * std::cos(kPiD * tm / 2) : 1.0;
        double r = speed(t) / c.radius;
        if (c.figure_eight) {
            const double lap = 2 * kPiD * c.radius / c.speed;
            r *= std::sin(kPiD * tm / lap) >= 0 ? 1 : -1;
            // Soften the reversal so lean rate stays finite.
            const double phase = std::fmod(tm, lap);
            const double edge = std::min(phase, lap - phase);
            if (edge < 1.5) r *= edge / 1.5;
        }
        return r * blend;
    }
    double lean(double t) const { return std::atan(speed(t) * yaw_rate(t) / kG); }
    static constexpr double kPiD = 3.14159265358979323846;
};

}  // namespace

Ride make_ride(const Config& c) {
    Ride ride;
    ride.frame = gins::LocalFrame(c.origin);
    Profile pr{c};
    std::mt19937 rng(c.seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    auto noise = [&](double s) { return c.noise ? s * n01(rng) : 0.0; };

    const Eigen::Vector3d ba(c.accel_bias, -0.6 * c.accel_bias, 0.8 * c.accel_bias);
    const Eigen::Vector3d bg(c.gyro_bias, -0.5 * c.gyro_bias, 0.7 * c.gyro_bias);

    const double total = c.static_s + c.ride_s;
    const double h = 1e-3;  // integration step
    const int imu_every = static_cast<int>(std::lround(1.0 / (c.imu_hz * h)));
    const int gnss_every = static_cast<int>(std::lround(1.0 / (c.gnss_hz * h)));
    const int steps = static_cast<int>(total / h);

    double psi = 0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    const double sa = c.accel_noise * std::sqrt(c.imu_hz);
    const double sg = c.gyro_noise * std::sqrt(c.imu_hz);

    for (int k = 1; k <= steps; ++k) {
        const double t0 = (k - 1) * h;
        const double t1 = k * h;
        const double tm = 0.5 * (t0 + t1);
        // Midpoint integration of heading and position.
        const double psi_m = psi + 0.5 * h * pr.yaw_rate(t0);
        p += h * pr.speed(tm) * Eigen::Vector3d(std::cos(psi_m), std::sin(psi_m), 0);
        psi += h * pr.yaw_rate(tm);

        if (k % imu_every == 0) {
            // Instantaneous kinematics at the middle of the IMU interval.
            const double dt_imu = 1.0 / c.imu_hz;
            const double t = t1 - 0.5 * dt_imu;
            const double v = pr.speed(t), r = pr.yaw_rate(t), phi = pr.lean(t);
            const double e = 1e-4;
            const double vdot = (pr.speed(t + e) - pr.speed(t - e)) / (2 * e);
            const double phidot = (pr.lean(t + e) - pr.lean(t - e)) / (2 * e);
            const double ps = psi - 0.5 * dt_imu * r;
            const Eigen::Vector3d a_n(vdot * std::cos(ps) - v * r * std::sin(ps),
                                      vdot * std::sin(ps) + v * r * std::cos(ps), 0);
            const Eigen::Matrix3d R =
                (Eigen::AngleAxisd(ps, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
            const Eigen::Vector3d f_b = R.transpose() * (a_n - Eigen::Vector3d(0, 0, kG));
            const Eigen::Vector3d w_b(phidot, r * std::sin(phi), r * std::cos(phi));

            gins::ImuSample s;
            s.t_us = kT0 + static_cast<int64_t>(std::llround(t1 * 1e6));
            for (int i = 0; i < 3; ++i) {
                const double vib = v > 0.5 ? noise(c.vibration) : 0.0;
                s.accel(i) = static_cast<gins::real>(f_b(i) + ba(i) + noise(sa) + vib);
                s.gyro(i) = static_cast<gins::real>(w_b(i) + bg(i) + noise(sg));
            }
            ride.imu.push_back(s);
        }
        if (k % gnss_every == 0) {
            const int64_t t_us = kT0 + static_cast<int64_t>(std::llround(t1 * 1e6));
            const gins::Lla truth = ride.frame.to_lla(p);
            ride.truth_t.push_back(t_us);
            ride.truth_lla.push_back(truth);
            ride.truth_ned.push_back(p);

            const Eigen::Vector3d pn = p + Eigen::Vector3d(noise(c.gnss_pos_sigma), noise(c.gnss_pos_sigma),
                                                           noise(1.5 * c.gnss_pos_sigma));
            const gins::Lla meas = ride.frame.to_lla(pn);
            const double v = pr.speed(t1);
            gins::GnssFix f;
            f.t_us = t_us;
            f.lat_deg = meas.lat_deg;
            f.lon_deg = meas.lon_deg;
            f.h_m = meas.h_m;
            f.vel_ned = gins::Vec3(static_cast<gins::real>(v * std::cos(psi) + noise(c.gnss_vel_sigma)),
                                   static_cast<gins::real>(v * std::sin(psi) + noise(c.gnss_vel_sigma)),
                                   static_cast<gins::real>(noise(c.gnss_vel_sigma)));
            f.h_acc = static_cast<gins::real>(std::max(c.gnss_pos_sigma, 0.05));
            f.v_acc = static_cast<gins::real>(std::max(1.5 * c.gnss_pos_sigma, 0.05));
            f.s_acc = static_cast<gins::real>(std::max(c.gnss_vel_sigma, 0.02));
            f.fix_type = 3;
            f.num_sv = 20;
            ride.fixes.push_back(f);
        }
    }
    return ride;
}

}  // namespace sim
