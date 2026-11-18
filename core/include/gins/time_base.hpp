// Maps the board's local microsecond clock to GPS time.
//
// The receiver's PPS pulse marks the start of each GPS second. The PPS ISR
// stamps the edge in local time; the next NAV-PVT for a whole second tells us
// which second it was. A least-squares line through the recent pairs gives
// offset and crystal drift. Before the first pair (or with no PPS wired) the
// fallback subtracts a nominal message latency from the NAV-PVT arrival time,
// which is the error this class exists to remove.
#pragma once

#include <array>
#include <cstdint>

namespace gins {

class TimeBase {
public:
    static constexpr std::size_t kPairs = 8;
    static constexpr int64_t kWeekUs = 604'800'000'000LL;

    explicit TimeBase(int64_t nominal_latency_us = 40'000);

    void on_pps(int64_t t_local_us);
    // NAV-PVT arrival (local time) and its epoch (GPS time of week, ms).
    // Returns the epoch as GPS time in microseconds.
    int64_t on_pvt(int64_t t_local_arrival_us, uint32_t itow_ms);

    // GPS time in microseconds (weeks since start counted, time of week).
    int64_t to_gps_us(int64_t t_local_us) const;
    // GPS time of a NAV-PVT epoch, with week rollover handled.
    int64_t epoch_gps_us(uint32_t itow_ms);

    bool locked(int64_t now_local_us) const;
    bool has_fit() const { return n_pairs_ > 0; }
    // Local clock rate error: positive when the board's crystal runs fast.
    double drift_ppm() const { return (1.0 / slope_ - 1.0) * 1e6; }
    // Mean NAV-PVT arrival latency after its epoch, measured against PPS time.
    double mean_latency_ms() const { return latency_n_ ? latency_sum_us_ / latency_n_ / 1000.0 : 0.0; }
    uint32_t pps_count() const { return pps_count_; }

private:
    void refit();

    int64_t nominal_latency_us_;
    int64_t pending_pps_ = -1;
    std::array<int64_t, kPairs> pair_local_{};
    std::array<int64_t, kPairs> pair_gps_{};
    std::size_t n_pairs_ = 0;
    std::size_t next_pair_ = 0;
    int64_t last_pair_local_ = 0;

    // Fit: gps = gps_ref_ + slope_ * (local - local_ref_)
    int64_t local_ref_ = 0;
    int64_t gps_ref_ = 0;
    double slope_ = 1.0;

    // Coarse fallback: gps = local + coarse_offset_
    bool have_coarse_ = false;
    int64_t coarse_offset_ = 0;

    uint32_t last_itow_ = 0;
    int64_t week_base_us_ = 0;
    bool have_itow_ = false;

    double latency_sum_us_ = 0;
    uint32_t latency_n_ = 0;
    uint32_t pps_count_ = 0;
};

}  // namespace gins
