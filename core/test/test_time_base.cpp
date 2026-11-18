#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "gins/time_base.hpp"

using gins::TimeBase;

namespace {
// Local clock runs 25 ppm fast and started at an arbitrary offset.
constexpr double kDrift = 25e-6;
constexpr int64_t kGps0 = 345'600'000'000LL;  // Wednesday 00:00 in GPS time of week
int64_t local_of(int64_t gps_us) {
    return 7'000'000 + static_cast<int64_t>(std::llround((gps_us - kGps0) * (1.0 + kDrift)));
}

void run_seconds(TimeBase& tb, int seconds, std::mt19937& rng, double latency_ms = 40.0) {
    std::normal_distribution<double> jitter(0.0, 2.0);  // us, PPS capture jitter
    std::normal_distribution<double> lat(latency_ms * 1000.0, 3000.0);
    for (int s = 0; s < seconds; ++s) {
        const int64_t sec = kGps0 + static_cast<int64_t>(s) * 1'000'000;
        tb.on_pps(local_of(sec) + static_cast<int64_t>(std::llround(jitter(rng))));
        for (int k = 0; k < 10; ++k) {
            const int64_t epoch = sec + k * 100'000;
            const int64_t arrival = local_of(epoch + static_cast<int64_t>(lat(rng)));
            tb.on_pvt(arrival, static_cast<uint32_t>(epoch / 1000));
        }
    }
}
}  // namespace

TEST(TimeBase, LocksToPpsAndTracksDrift) {
    std::mt19937 rng(1);
    TimeBase tb;
    run_seconds(tb, 20, rng);
    const int64_t now = local_of(kGps0 + 20'000'000);
    EXPECT_TRUE(tb.locked(now));
    EXPECT_NEAR(tb.drift_ppm(), 25.0, 1.0);
    // Any local time maps back to GPS time within a few microseconds.
    for (int64_t g = kGps0 + 15'000'000; g < kGps0 + 20'000'000; g += 123'457) {
        EXPECT_NEAR(static_cast<double>(tb.to_gps_us(local_of(g)) - g), 0.0, 5.0);
    }
    EXPECT_NEAR(tb.mean_latency_ms(), 40.0, 1.0);
}

TEST(TimeBase, ArrivalStampingIsOffByTheLatency) {
    // Without PPS the fallback assumes a 40 ms latency. A receiver that is
    // actually 55 ms late leaves every timestamp 15 ms wrong: the bug PPS fixes.
    TimeBase tb(40'000);
    const int64_t epoch = kGps0;
    tb.on_pvt(local_of(epoch + 55'000), static_cast<uint32_t>(epoch / 1000));
    const int64_t err = tb.to_gps_us(local_of(epoch)) - epoch;
    EXPECT_NEAR(static_cast<double>(err), -15'000.0, 50.0);
    EXPECT_FALSE(tb.has_fit());
}

TEST(TimeBase, LosesLockWithoutPps) {
    std::mt19937 rng(2);
    TimeBase tb;
    run_seconds(tb, 5, rng);
    EXPECT_TRUE(tb.locked(local_of(kGps0 + 5'000'000)));
    EXPECT_FALSE(tb.locked(local_of(kGps0 + 9'000'000)));
    EXPECT_TRUE(tb.has_fit());  // still in holdover on the last fit
}

TEST(TimeBase, HandlesGpsWeekRollover) {
    TimeBase tb;
    const int64_t a = tb.epoch_gps_us(604'799'900);  // last epoch of the week
    const int64_t b = tb.epoch_gps_us(0);            // first epoch of the next
    EXPECT_EQ(b - a, 100'000);
}
