#include "gins/navigator.hpp"

#include <cmath>

#include "gins/rotation.hpp"

namespace gins {

Navigator::Navigator(const NavConfig& cfg)
    : cfg_(cfg), filter_(cfg.filter), aligner_(cfg.align_us) {}

void Navigator::push_imu(const ImuSample& s) {
    if (imu_q_.full()) {
        // The consumer fell behind: process the oldest item now rather than drop it.
        ++diag_.imu_overflow;
        step();
    }
    imu_q_.push(s);
    if (s.t_us > newest_imu_t_) {
        newest_imu_t_ = s.t_us;
    }
}

void Navigator::push_fix(const GnssFix& f) {
    if (fix_q_.full()) {
        ++diag_.fix_overflow;
        step();
    }
    fix_q_.push(f);
}

void Navigator::process() {
    const int64_t horizon = newest_imu_t_ - cfg_.lag_us;
    while (true) {
        int64_t next = INT64_MAX;
        if (!imu_q_.empty()) next = imu_q_.front().t_us;
        if (!fix_q_.empty() && fix_q_.front().t_us < next) next = fix_q_.front().t_us;
        if (next == INT64_MAX || next > horizon) break;
        step();
    }
}

void Navigator::flush() {
    while (!imu_q_.empty() || !fix_q_.empty()) {
        step();
    }
}

void Navigator::step() {
    const bool take_fix =
        !fix_q_.empty() && (imu_q_.empty() || fix_q_.front().t_us <= imu_q_.front().t_us);
    if (take_fix) {
        const GnssFix f = fix_q_.front();
        fix_q_.pop();
        handle_fix(f);
    } else if (!imu_q_.empty()) {
        const ImuSample s = imu_q_.front();
        imu_q_.pop();
        handle_imu(s);
    }
}

bool Navigator::add_gnss_mask(int64_t start_us, int64_t end_us) {
    if (n_masks_ >= kMaxMasks || end_us <= start_us) {
        return false;
    }
    masks_[2 * n_masks_] = start_us;
    masks_[2 * n_masks_ + 1] = end_us;
    ++n_masks_;
    return true;
}

bool Navigator::masked(int64_t t) const {
    for (std::size_t i = 0; i < n_masks_; ++i) {
        if (t >= masks_[2 * i] && t < masks_[2 * i + 1]) return true;
    }
    return false;
}

void Navigator::restart_alignment() {
    aligner_.reset();
    yaw_run_start_ = 0;
    mode_ = frame_.valid() ? Mode::kAligning : Mode::kWaitFix;
    ++diag_.align_restarts;
}

void Navigator::update_still(const ImuSample& s) {
    still_norm_[still_idx_] = s.accel.norm();
    still_gyro_[still_idx_] = s.gyro;
    still_idx_ = (still_idx_ + 1) % kStillWindow;
    if (still_n_ < kStillWindow) ++still_n_;
    if (still_n_ < kStillWindow) {
        stationary_ = false;
        return;
    }
    real mean = 0;
    Vec3 gsum = Vec3::Zero();
    for (std::size_t i = 0; i < kStillWindow; ++i) {
        mean += still_norm_[i];
        gsum += still_gyro_[i];
    }
    mean /= static_cast<real>(kStillWindow);
    real var = 0;
    for (std::size_t i = 0; i < kStillWindow; ++i) {
        const real d = still_norm_[i] - mean;
        var += d * d;
    }
    var /= static_cast<real>(kStillWindow - 1);
    still_gyro_mean_ = gsum / static_cast<real>(kStillWindow);
    const Vec3 rate = still_gyro_mean_ - filter_.state().bg;
    stationary_ = std::sqrt(var) < cfg_.still_accel_std && rate.norm() < cfg_.still_gyro;
}

void Navigator::handle_imu(const ImuSample& s) {
    last_imu_ = s;
    have_imu_ = true;
    update_still(s);

    switch (mode_) {
        case Mode::kWaitFix:
        case Mode::kAligning: {
            // Alignment can finish before the first good fix; the fix then moves
            // straight on to waiting for yaw.
            const StaticAligner::Status st = aligner_.add(s);
            if (st == StaticAligner::Status::kMoved) {
                ++diag_.align_restarts;
            } else if (st == StaticAligner::Status::kDone && mode_ == Mode::kAligning) {
                mode_ = Mode::kWaitYaw;
            }
            return;
        }
        case Mode::kWaitYaw:
            return;
        case Mode::kNav:
        case Mode::kDegraded:
            break;
    }

    filter_.propagate(s);

    if (s.t_us - last_aux_t_ >= cfg_.aux_period_us) {
        last_aux_t_ = s.t_us;
        if (stationary_) {
            filter_.update_zupt(still_gyro_mean_);
        } else {
            filter_.update_nhc();
        }
    }
    if (mode_ == Mode::kNav && s.t_us - last_accept_t_ > cfg_.degraded_after_us) {
        mode_ = Mode::kDegraded;
    }
    if (s.t_us - last_out_t_ >= cfg_.output_period_us) {
        last_out_t_ = s.t_us;
        emit(s.t_us);
    }
}

void Navigator::init_filter(const GnssFix& f, real roll, real pitch, const Vec3& bg, bool coarse) {
    State x;
    x.t_us = f.t_us;
    x.p = frame_.to_ned({f.lat_deg, f.lon_deg, f.h_m}).cast<real>();
    x.v = f.vel_ned;
    const real yaw = std::atan2(f.vel_ned.y(), f.vel_ned.x());
    x.q = quat_from_rpy(roll, pitch, yaw);
    x.bg = bg;

    const real att = static_cast<real>((coarse ? 5.0 : 2.0) * kDeg);
    ErrVec sig;
    sig << f.h_acc, f.h_acc, f.v_acc,
           f.s_acc * 2 + static_cast<real>(0.1), f.s_acc * 2 + static_cast<real>(0.1),
           f.s_acc * 2 + static_cast<real>(0.1),
           att, att, static_cast<real>(10.0 * kDeg),
           static_cast<real>(0.2), static_cast<real>(0.2), static_cast<real>(0.2),
           coarse ? static_cast<real>(0.01) : static_cast<real>(0.003),
           coarse ? static_cast<real>(0.01) : static_cast<real>(0.003),
           coarse ? static_cast<real>(0.01) : static_cast<real>(0.003);
    filter_.init(x, sig);
    filter_.set_gravity(static_cast<real>(normal_gravity(f.lat_deg, f.h_m)));
    mode_ = Mode::kNav;
    last_accept_t_ = f.t_us;
    last_aux_t_ = f.t_us;
    last_out_t_ = 0;
    rejects_in_row_ = 0;
}

void Navigator::handle_fix(const GnssFix& f) {
    if (masked(f.t_us)) {
        ++diag_.fixes_masked;
        return;
    }
    if (f.fix_type < 3 || f.h_acc > cfg_.max_hacc) {
        ++diag_.fixes_ignored;
        return;
    }

    if (mode_ == Mode::kWaitFix) {
        if (f.h_acc <= cfg_.origin_max_hacc) {
            frame_ = LocalFrame({f.lat_deg, f.lon_deg, f.h_m});
            mode_ = aligner_.status() == StaticAligner::Status::kDone ? Mode::kWaitYaw
                                                                       : Mode::kAligning;
        }
        return;
    }

    if (mode_ == Mode::kAligning || mode_ == Mode::kWaitYaw) {
        const real speed = std::hypot(f.vel_ned.x(), f.vel_ned.y());
        if (speed < cfg_.yaw_speed) {
            yaw_run_start_ = 0;
            return;
        }
        if (yaw_run_start_ == 0) yaw_run_start_ = f.t_us;
        if (f.t_us - yaw_run_start_ < cfg_.yaw_hold_us) return;

        if (mode_ == Mode::kWaitYaw) {
            init_filter(f, aligner_.roll(), aligner_.pitch(), aligner_.gyro_bias(), false);
        } else {
            // Moving before static alignment finished: start level with zero gyro
            // bias and wider attitude uncertainty.
            ++diag_.coarse_alignments;
            init_filter(f, 0, 0, Vec3::Zero(), true);
        }
        return;
    }

    // Navigating.
    if (f.t_us < filter_.state().t_us) {
        ++diag_.fixes_late;
        return;
    }
    // Bring the filter exactly to the fix epoch with the next IMU sample's reading,
    // which covers the interval the epoch falls in.
    const ImuSample& next = imu_q_.empty() ? last_imu_ : imu_q_.front();
    if (have_imu_) {
        filter_.propagate_to(f.t_us, next.accel, next.gyro);
    }

    const Vec3 p = frame_.to_ned({f.lat_deg, f.lon_deg, f.h_m}).cast<real>();
    if (filter_.update_gnss(p, f.vel_ned, f.h_acc, f.v_acc, f.s_acc)) {
        rejects_in_row_ = 0;
        last_accept_t_ = f.t_us;
        mode_ = Mode::kNav;
        return;
    }
    if (++rejects_in_row_ >= cfg_.max_consecutive_rejects) {
        // Ten good-looking fixes in a row disagree with the filter: trust them.
        const State& x = filter_.state();
        filter_.inflate_pos_vel(p - x.p, f.vel_ned - x.v);
        filter_.update_gnss(p, f.vel_ned, f.h_acc, f.v_acc, f.s_acc, true);
        ++diag_.forced_resets;
        rejects_in_row_ = 0;
        last_accept_t_ = f.t_us;
        mode_ = Mode::kNav;
    }
}

NavOutput Navigator::current() const {
    NavOutput o;
    const State& x = filter_.state();
    o.t_us = x.t_us;
    o.mode = mode_;
    o.stationary = stationary_;
    if (frame_.valid()) {
        o.lla = frame_.to_lla(x.p.cast<double>());
    }
    o.v_ned = x.v;
    o.rpy = rpy_from_quat(x.q);
    const ErrMat& P = filter_.cov();
    o.sigma_pos = Vec3(std::sqrt(P(0, 0)), std::sqrt(P(1, 1)), std::sqrt(P(2, 2)));
    o.gnss_masked = masked(x.t_us);
    return o;
}

void Navigator::emit(int64_t) {
    if (out_fn_) {
        out_fn_(current(), out_ctx_);
    }
}

}  // namespace gins
