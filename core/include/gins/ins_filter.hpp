// Loosely coupled error-state Kalman filter for GNSS/INS.
//
// Nominal state: position and velocity in local NED, body to NED attitude,
// accelerometer and gyro biases. Error state (15): dp, dv, dtheta (body frame),
// dba, dbg. Earth rotation and transport rate are ignored: at bike speeds and
// MEMS gyro noise levels they are far below the noise floor.
#pragma once

#include "gins/types.hpp"

namespace gins {

class InsFilter {
public:
    explicit InsFilter(const FilterConfig& cfg = FilterConfig{});

    void init(const State& s, const ErrVec& sigma0);
    void set_gravity(real g) { gravity_ = g; }

    // Propagate with one IMU sample, from the current state time to imu.t_us.
    void propagate(const ImuSample& imu);
    // Propagate to t_us holding accel and gyro constant over the interval.
    void propagate_to(int64_t t_us, const Vec3& accel, const Vec3& gyro);

    // GNSS position (NED, m) and velocity (NED, m/s) update. Returns true when
    // the update was applied. force skips the innovation gate.
    bool update_gnss(const Vec3& p_ned, const Vec3& v_ned, real h_acc, real v_acc,
                     real s_acc, bool force = false);
    // Bike motion constraint: body-frame lateral and vertical velocity are zero.
    bool update_nhc();
    // Zero velocity and zero angular rate update while stationary.
    bool update_zupt(const Vec3& gyro_meas);

    // Inflate position and velocity covariance so a large jump can be accepted.
    void inflate_pos_vel(const Vec3& dp, const Vec3& dv);

    const State& state() const { return x_; }
    const ErrMat& cov() const { return P_; }
    const FilterDiag& diag() const { return diag_; }
    const FilterConfig& config() const { return cfg_; }
    FilterConfig& config() { return cfg_; }

    // Last normalized innovation squared of a GNSS update (accepted or not).
    real last_gnss_nis() const { return diag_.last_gnss_nis; }

private:
    template <int M>
    bool kalman_update(const Eigen::Matrix<real, M, 1>& y,
                       const Eigen::Matrix<real, M, kErr>& H,
                       const Eigen::Matrix<real, M, M>& R, real gate, real* nis_out);
    void inject(const ErrVec& dx);
    void symmetrize();

    FilterConfig cfg_;
    State x_;
    ErrMat P_ = ErrMat::Identity();
    real gravity_ = static_cast<real>(9.80665);
    FilterDiag diag_;
};

}  // namespace gins
