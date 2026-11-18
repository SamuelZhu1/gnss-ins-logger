#include <gtest/gtest.h>

#include <random>

#include "gins/alignment.hpp"
#include "gins/rotation.hpp"

using namespace gins;

namespace {
ImuSample still_sample(int64_t t, real roll, real pitch, const Vec3& bg, std::mt19937& rng) {
    std::normal_distribution<double> n(0, 1);
    const Mat3 R = quat_from_rpy(roll, pitch, static_cast<real>(0.7)).toRotationMatrix();
    ImuSample s;
    s.t_us = t;
    s.accel = R.transpose() * Vec3(0, 0, static_cast<real>(-9.81));
    for (int i = 0; i < 3; ++i) {
        s.accel(i) += static_cast<real>(0.02 * n(rng));
        s.gyro(i) = bg(i) + static_cast<real>(0.002 * n(rng));
    }
    return s;
}
}  // namespace

TEST(StaticAligner, RecoversTiltAndGyroBias) {
    std::mt19937 rng(3);
    const real roll = static_cast<real>(3.0 * kDeg), pitch = static_cast<real>(-2.0 * kDeg);
    const Vec3 bg(static_cast<real>(0.004), static_cast<real>(-0.002), static_cast<real>(0.003));
    StaticAligner a(5'000'000);
    StaticAligner::Status st = StaticAligner::Status::kCollecting;
    for (int k = 0; k <= 1000 && st != StaticAligner::Status::kDone; ++k) {
        st = a.add(still_sample(k * 5000, roll, pitch, bg, rng));
    }
    ASSERT_EQ(st, StaticAligner::Status::kDone);
    EXPECT_NEAR(a.roll(), roll, 0.05 * kDeg);
    EXPECT_NEAR(a.pitch(), pitch, 0.05 * kDeg);
    EXPECT_LT((a.gyro_bias() - bg).norm(), 3e-4);
}

TEST(StaticAligner, RestartsWhenTheBoardMoves) {
    std::mt19937 rng(4);
    StaticAligner a(5'000'000);
    for (int k = 0; k < 400; ++k) a.add(still_sample(k * 5000, 0, 0, Vec3::Zero(), rng));
    ImuSample bump = still_sample(400 * 5000, 0, 0, Vec3::Zero(), rng);
    bump.gyro.z() = static_cast<real>(0.5);  // someone turns the handlebars
    EXPECT_EQ(a.add(bump), StaticAligner::Status::kMoved);
    EXPECT_EQ(a.elapsed_us(), 0);
}
