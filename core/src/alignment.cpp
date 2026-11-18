#include "gins/alignment.hpp"

#include <cmath>

namespace gins {

StaticAligner::StaticAligner(int64_t duration_us, real max_accel_std, real max_gyro)
    : duration_us_(duration_us), max_accel_std_(max_accel_std), max_gyro_(max_gyro) {}

void StaticAligner::reset() {
    status_ = Status::kCollecting;
    n_ = 0;
    accel_sum_.setZero();
    gyro_sum_.setZero();
    norm_mean_ = 0;
    norm_m2_ = 0;
}

StaticAligner::Status StaticAligner::add(const ImuSample& s) {
    if (status_ == Status::kDone) {
        return status_;
    }
    if (n_ == 0) {
        first_t_ = s.t_us;
    }
    last_t_ = s.t_us;
    ++n_;
    accel_sum_ += s.accel;
    gyro_sum_ += s.gyro;

    const double x = s.accel.norm();
    const double delta = x - norm_mean_;
    norm_mean_ += delta / n_;
    norm_m2_ += delta * (x - norm_mean_);

    // Check for motion once there are enough samples for a stable variance.
    if (n_ > 20) {
        const double sd = std::sqrt(norm_m2_ / (n_ - 1));
        if (sd > max_accel_std_ || s.gyro.norm() > max_gyro_ * 4) {
            reset();
            status_ = Status::kMoved;
            return status_;
        }
    }
    status_ = Status::kCollecting;

    if (last_t_ - first_t_ >= duration_us_) {
        const Vec3 f = accel_sum_ / static_cast<real>(n_);
        gyro_bias_ = gyro_sum_ / static_cast<real>(n_);
        if (gyro_bias_.norm() > max_gyro_) {
            // Rotating steadily: not static.
            reset();
            status_ = Status::kMoved;
            return status_;
        }
        // At rest the accelerometer measures f_b = R^T (0, 0, -g).
        roll_ = std::atan2(-f.y(), -f.z());
        pitch_ = std::atan2(f.x(), std::sqrt(f.y() * f.y() + f.z() * f.z()));
        status_ = Status::kDone;
    }
    return status_;
}

}  // namespace gins
