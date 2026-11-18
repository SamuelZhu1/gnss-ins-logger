// Navigator: runs the filter on time-ordered IMU and GNSS data.
//
// GNSS fixes arrive 20 to 60 ms after their epoch, so the navigator holds IMU
// samples for a fixed lag and merges both streams by GPS time. A fix whose
// epoch falls between two IMU samples is applied at its exact time. The same
// class runs on the ESP32 (live) and on the host (replay of logged rides).
#pragma once

#include <array>
#include <cstdint>

#include "gins/alignment.hpp"
#include "gins/geo.hpp"
#include "gins/ins_filter.hpp"
#include "gins/ring_queue.hpp"
#include "gins/types.hpp"

namespace gins {

struct NavConfig {
    FilterConfig filter;
    int64_t lag_us = 100'000;           // hold IMU this long so late fixes slot in
    int64_t align_us = 20'000'000;      // static alignment length
    real origin_max_hacc = 5;           // fix quality needed to set the local origin
    real max_hacc = 20;                 // fixes worse than this are ignored
    real yaw_speed = 3;                 // m/s needed to take yaw from GNSS course
    int64_t yaw_hold_us = 3'000'000;
    int64_t degraded_after_us = 60'000'000;
    int max_consecutive_rejects = 10;   // then accept the next fix with inflated covariance
    int64_t output_period_us = 100'000;
    int64_t aux_period_us = 100'000;    // bike motion constraint and ZUPT rate
    real still_accel_std = static_cast<real>(0.25);  // m/s^2, accel norm over the window
    real still_gyro = static_cast<real>(0.03);       // rad/s, mean gyro norm
};

struct NavOutput {
    int64_t t_us = 0;
    Lla lla;
    Vec3 v_ned = Vec3::Zero();
    Vec3 rpy = Vec3::Zero();
    Vec3 sigma_pos = Vec3::Zero();  // 1 sigma N, E, D
    Mode mode = Mode::kWaitFix;
    bool gnss_masked = false;
    bool stationary = false;
};

struct NavDiag {
    uint32_t imu_overflow = 0;
    uint32_t fix_overflow = 0;
    uint32_t fixes_masked = 0;
    uint32_t fixes_ignored = 0;
    uint32_t fixes_late = 0;
    uint32_t align_restarts = 0;
    uint32_t coarse_alignments = 0;
    uint32_t forced_resets = 0;
};

using NavOutputFn = void (*)(const NavOutput& out, void* ctx);

class Navigator {
public:
    static constexpr std::size_t kImuQueue = 128;
    static constexpr std::size_t kFixQueue = 16;
    static constexpr std::size_t kMaxMasks = 64;
    static constexpr std::size_t kStillWindow = 40;

    explicit Navigator(const NavConfig& cfg = NavConfig{});

    void push_imu(const ImuSample& s);
    void push_fix(const GnssFix& f);
    // Process everything older than (newest IMU time - lag).
    void process();
    // Process everything that is queued (end of a replay).
    void flush();

    void set_output(NavOutputFn fn, void* ctx) {
        out_fn_ = fn;
        out_ctx_ = ctx;
    }
    // Ignore GNSS fixes with epochs in [start, end) to simulate an outage.
    bool add_gnss_mask(int64_t start_us, int64_t end_us);
    void clear_gnss_masks() { n_masks_ = 0; }
    void restart_alignment();

    Mode mode() const { return mode_; }
    const InsFilter& filter() const { return filter_; }
    const LocalFrame& frame() const { return frame_; }
    const NavDiag& diag() const { return diag_; }
    const NavConfig& config() const { return cfg_; }
    bool stationary() const { return stationary_; }
    NavOutput current() const;

private:
    void step();
    void handle_imu(const ImuSample& s);
    void handle_fix(const GnssFix& f);
    void init_filter(const GnssFix& f, real roll, real pitch, const Vec3& bg, bool coarse);
    void update_still(const ImuSample& s);
    bool masked(int64_t t) const;
    void emit(int64_t t);

    NavConfig cfg_;
    InsFilter filter_;
    StaticAligner aligner_;
    LocalFrame frame_;
    Mode mode_ = Mode::kWaitFix;
    NavDiag diag_;

    RingQueue<ImuSample, kImuQueue> imu_q_;
    RingQueue<GnssFix, kFixQueue> fix_q_;
    int64_t newest_imu_t_ = 0;
    ImuSample last_imu_;
    bool have_imu_ = false;

    std::array<int64_t, kMaxMasks * 2> masks_{};
    std::size_t n_masks_ = 0;

    int64_t yaw_run_start_ = 0;
    int64_t last_accept_t_ = 0;
    int64_t last_aux_t_ = 0;
    int64_t last_out_t_ = 0;
    int rejects_in_row_ = 0;

    // Stationary detector over the last kStillWindow samples.
    std::array<real, kStillWindow> still_norm_{};
    std::array<Vec3, kStillWindow> still_gyro_{};
    std::size_t still_idx_ = 0;
    std::size_t still_n_ = 0;
    bool stationary_ = false;
    Vec3 still_gyro_mean_ = Vec3::Zero();

    NavOutputFn out_fn_ = nullptr;
    void* out_ctx_ = nullptr;
};

}  // namespace gins
