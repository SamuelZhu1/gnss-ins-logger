#include "gins/time_base.hpp"

#include <cmath>

namespace gins {

TimeBase::TimeBase(int64_t nominal_latency_us) : nominal_latency_us_(nominal_latency_us) {}

void TimeBase::on_pps(int64_t t_local_us) {
    pending_pps_ = t_local_us;
    ++pps_count_;
}

int64_t TimeBase::epoch_gps_us(uint32_t itow_ms) {
    if (have_itow_ && itow_ms + 302'400'000u < last_itow_) {
        week_base_us_ += kWeekUs;  // time of week wrapped at the end of the GPS week
    }
    last_itow_ = itow_ms;
    have_itow_ = true;
    return week_base_us_ + static_cast<int64_t>(itow_ms) * 1000;
}

int64_t TimeBase::on_pvt(int64_t t_local_arrival_us, uint32_t itow_ms) {
    const int64_t epoch = epoch_gps_us(itow_ms);

    // Measure latency against the PPS fit before this message changes it.
    if (n_pairs_ > 0) {
        latency_sum_us_ += static_cast<double>(to_gps_us(t_local_arrival_us) - epoch);
        ++latency_n_;
    }

    if (!have_coarse_ || n_pairs_ == 0) {
        coarse_offset_ = epoch - (t_local_arrival_us - nominal_latency_us_);
        have_coarse_ = true;
    }

    // A whole-second epoch arriving shortly after a PPS edge names that edge.
    if (itow_ms % 1000 == 0 && pending_pps_ >= 0) {
        const int64_t gap = t_local_arrival_us - pending_pps_;
        if (gap > 0 && gap < 500'000) {
            pair_local_[next_pair_] = pending_pps_;
            pair_gps_[next_pair_] = epoch;
            next_pair_ = (next_pair_ + 1) % kPairs;
            if (n_pairs_ < kPairs) ++n_pairs_;
            last_pair_local_ = pending_pps_;
            refit();
        }
        pending_pps_ = -1;
    }
    return epoch;
}

void TimeBase::refit() {
    // Least squares on values relative to the newest pair keeps the numbers small.
    const std::size_t newest = (next_pair_ + kPairs - 1) % kPairs;
    local_ref_ = pair_local_[newest];
    gps_ref_ = pair_gps_[newest];
    if (n_pairs_ < 2) {
        slope_ = 1.0;
        return;
    }
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (std::size_t i = 0; i < n_pairs_; ++i) {
        const double x = static_cast<double>(pair_local_[i] - local_ref_);
        const double y = static_cast<double>(pair_gps_[i] - gps_ref_);
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const double n = static_cast<double>(n_pairs_);
    const double den = n * sxx - sx * sx;
    if (std::fabs(den) < 1e-3) {
        slope_ = 1.0;
        return;
    }
    const double s = (n * sxy - sx * sy) / den;
    // A crystal is good to well under 1000 ppm; anything else is a bad pair.
    if (std::fabs(s - 1.0) < 1e-3) {
        slope_ = s;
        // Re-anchor at the fitted line through the mean point.
        const double b = (sy - s * sx) / n;
        gps_ref_ += static_cast<int64_t>(std::llround(b));
    }
}

int64_t TimeBase::to_gps_us(int64_t t_local_us) const {
    if (n_pairs_ > 0) {
        const double dx = static_cast<double>(t_local_us - local_ref_);
        return gps_ref_ + static_cast<int64_t>(std::llround(slope_ * dx));
    }
    return t_local_us + coarse_offset_;
}

bool TimeBase::locked(int64_t now_local_us) const {
    return n_pairs_ > 0 && now_local_us - last_pair_local_ < 3'000'000;
}

}  // namespace gins
