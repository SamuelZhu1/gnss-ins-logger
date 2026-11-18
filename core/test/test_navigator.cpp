#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>

#include "gins/navigator.hpp"
#include "sim.hpp"

using namespace gins;

namespace {

struct Track {
    std::map<int64_t, NavOutput> out;
    static void collect(const NavOutput& o, void* ctx) { static_cast<Track*>(ctx)->out[o.t_us] = o; }
};

// Feed a synthetic ride in arrival order: fixes arrive 40 ms after their epoch.
Track run(const sim::Ride& ride, NavConfig cfg, std::vector<std::pair<int64_t, int64_t>> masks = {},
          Navigator* keep = nullptr) {
    Track tr;
    Navigator local(cfg);
    Navigator& nav = keep ? *keep : local;
    nav.set_output(Track::collect, &tr);
    for (const auto& m : masks) nav.add_gnss_mask(m.first, m.second);
    std::size_t j = 0;
    for (const ImuSample& s : ride.imu) {
        while (j < ride.fixes.size() && ride.fixes[j].t_us + 40'000 <= s.t_us) {
            nav.push_fix(ride.fixes[j++]);
        }
        nav.push_imu(s);
        nav.process();
    }
    nav.flush();
    return tr;
}

double horiz_error_at(const sim::Ride& ride, const Track& tr, int64_t t) {
    const auto it = std::lower_bound(ride.truth_t.begin(), ride.truth_t.end(), t);
    if (it == ride.truth_t.end()) return 1e9;
    const std::size_t k = static_cast<std::size_t>(it - ride.truth_t.begin());
    // Nearest output at or after t.
    const auto o = tr.out.lower_bound(t);
    if (o == tr.out.end()) return 1e9;
    const Eigen::Vector3d est = ride.frame.to_ned(o->second.lla);
    // Truth at the output time (outputs land on IMU times, within 5 ms of the fix time).
    const double dt = (o->first - ride.truth_t[k]) * 1e-6;
    const Eigen::Vector3d next = ride.truth_ned[std::min(k + 1, ride.truth_ned.size() - 1)];
    const Eigen::Vector3d truth = ride.truth_ned[k] + (next - ride.truth_ned[k]) * (dt / 0.1);
    return std::hypot(est.x() - truth.x(), est.y() - truth.y());
}

}  // namespace

TEST(Navigator, AlignsThenNavigatesOnACircle) {
    sim::Config c;
    const sim::Ride ride = sim::make_ride(c);
    Navigator nav;
    const Track tr = run(ride, NavConfig{}, {}, &nav);
    EXPECT_EQ(nav.mode(), Mode::kNav);
    EXPECT_EQ(nav.diag().coarse_alignments, 0u);
    ASSERT_FALSE(tr.out.empty());

    // RMS horizontal error over the last 60 s against truth.
    const int64_t t_end = ride.truth_t.back();
    double sum = 0;
    int n = 0;
    for (int64_t t = t_end - 60'000'000; t < t_end - 200'000; t += 500'000) {
        const double e = horiz_error_at(ride, tr, t);
        sum += e * e;
        ++n;
    }
    EXPECT_LT(std::sqrt(sum / n), 0.6);
    EXPECT_GT(nav.filter().diag().nhc_updates, 100u);

    // Gyro bias estimate converges toward the simulated bias.
    EXPECT_NEAR(nav.filter().state().bg.z(), 0.7 * c.gyro_bias, 0.0015);
}

TEST(Navigator, BikeConstraintCutsOutageDrift) {
    sim::Config c;
    c.ride_s = 150;
    c.figure_eight = true;
    const sim::Ride ride = sim::make_ride(c);
    const int64_t start = ride.truth_t.front() + static_cast<int64_t>((c.static_s + 90) * 1e6);
    const int64_t end = start + 20'000'000;

    NavConfig with;
    NavConfig without;
    without.filter.use_nhc = false;
    const Track a = run(ride, with, {{start, end}});
    const Track b = run(ride, without, {{start, end}});
    const double ea = horiz_error_at(ride, a, end - 100'000);
    const double eb = horiz_error_at(ride, b, end - 100'000);
    // Synthetic numbers, not ride results: only the ordering and a loose bound matter.
    EXPECT_LT(ea, eb);
    EXPECT_LT(ea, 15.0);
    EXPECT_TRUE(a.out.lower_bound(start + 1'000'000)->second.gnss_masked);
}

TEST(Navigator, FixBetweenSamplesIsAppliedAtItsEpoch) {
    sim::Config c;
    c.ride_s = 40;
    sim::Ride ride = sim::make_ride(c);
    // Shift every fix 2.5 ms so epochs fall between IMU samples.
    for (GnssFix& f : ride.fixes) f.t_us += 2500;
    for (int64_t& t : ride.truth_t) t += 2500;
    Navigator nav;
    run(ride, NavConfig{}, {}, &nav);
    EXPECT_EQ(nav.mode(), Mode::kNav);
    EXPECT_EQ(nav.diag().fixes_late, 0u);
    // A 6 DOF gate at 99.9 % rejects about 1 fix in 1000 by chance; allow a few.
    const auto& d = nav.filter().diag();
    EXPECT_LE(d.gnss_rejected, 5u);
    EXPECT_GT(d.gnss_accepted, 300u);
}

TEST(Navigator, StampingFixesOnArrivalHurtsAccuracy) {
    // The bug PPS time tagging fixes: label each fix with its arrival time
    // (40 ms late) and the track lags the bike, worst in turns.
    sim::Config c;
    c.ride_s = 120;
    c.noise = false;
    const sim::Ride ride = sim::make_ride(c);
    sim::Ride late = ride;
    for (GnssFix& f : late.fixes) f.t_us += 40'000;

    auto rms = [&](const Track& tr) {
        const int64_t t_end = ride.truth_t.back();
        double sum = 0;
        int n = 0;
        for (int64_t t = t_end - 60'000'000; t < t_end - 200'000; t += 500'000) {
            const double e = horiz_error_at(ride, tr, t);
            sum += e * e;
            ++n;
        }
        return std::sqrt(sum / n);
    };
    const double good = rms(run(ride, NavConfig{}));
    const double bad = rms(run(late, NavConfig{}));
    EXPECT_GT(bad, 0.15);        // 6 m/s x 40 ms = 0.24 m behind the bike
    EXPECT_LT(good, bad / 3);
}

TEST(Navigator, FixOlderThanTheLagIsDroppedAsLate) {
    sim::Config c;
    c.ride_s = 40;
    const sim::Ride ride = sim::make_ride(c);
    Navigator nav;
    run(ride, NavConfig{}, {}, &nav);
    ASSERT_EQ(nav.mode(), Mode::kNav);
    GnssFix old = ride.fixes[ride.fixes.size() / 2];
    nav.push_fix(old);
    nav.flush();
    EXPECT_EQ(nav.diag().fixes_late, 1u);
}

TEST(Navigator, StartsCoarseWhenRidingBeforeAlignment) {
    sim::Config c;
    c.static_s = 3;  // not long enough to align
    c.ride_s = 60;
    const sim::Ride ride = sim::make_ride(c);
    Navigator nav;
    run(ride, NavConfig{}, {}, &nav);
    EXPECT_EQ(nav.mode(), Mode::kNav);
    EXPECT_EQ(nav.diag().coarse_alignments, 1u);
}

TEST(Navigator, RecoversAfterLongOutageWithForcedReset) {
    sim::Config c;
    c.ride_s = 200;
    c.gyro_bias = 0.02;  // a bad gyro so the outage drift is large
    const sim::Ride ride = sim::make_ride(c);
    const int64_t start = ride.truth_t.front() + static_cast<int64_t>((c.static_s + 60) * 1e6);
    NavConfig cfg;
    cfg.filter.use_nhc = false;
    cfg.filter.use_zupt = false;
    Navigator nav(cfg);
    const Track tr = run(ride, cfg, {{start, start + 90'000'000}}, &nav);
    EXPECT_EQ(nav.mode(), Mode::kNav);
    EXPECT_LT(horiz_error_at(ride, tr, ride.truth_t.back() - 1'000'000), 2.0);
}
