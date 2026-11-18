// Static alignment: with the bike held still, the mean accelerometer reading
// gives roll and pitch, and the mean gyro reading gives the gyro bias.
#pragma once

#include "gins/types.hpp"

namespace gins {

class StaticAligner {
public:
    enum class Status { kCollecting, kDone, kMoved };

    explicit StaticAligner(int64_t duration_us = 20'000'000,
                           real max_accel_std = static_cast<real>(0.15),
                           real max_gyro = static_cast<real>(0.05));

    // Feed one sample. Returns kMoved (and restarts) if the board moved.
    Status add(const ImuSample& s);
    Status status() const { return status_; }
    void reset();

    real roll() const { return roll_; }
    real pitch() const { return pitch_; }
    const Vec3& gyro_bias() const { return gyro_bias_; }
    int64_t elapsed_us() const { return n_ ? last_t_ - first_t_ : 0; }

private:
    int64_t duration_us_;
    real max_accel_std_;
    real max_gyro_;
    Status status_ = Status::kCollecting;
    uint32_t n_ = 0;
    int64_t first_t_ = 0;
    int64_t last_t_ = 0;
    Vec3 accel_sum_ = Vec3::Zero();
    Vec3 gyro_sum_ = Vec3::Zero();
    // Welford running variance of the accel norm.
    double norm_mean_ = 0;
    double norm_m2_ = 0;
    real roll_ = 0;
    real pitch_ = 0;
    Vec3 gyro_bias_ = Vec3::Zero();
};

}  // namespace gins
