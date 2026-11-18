#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <random>

#include "gins/ins_filter.hpp"
#include "gins/rotation.hpp"

using namespace gins;

namespace {

ErrVec default_sigma() {
    ErrVec s;
    s << 1, 1, 1, 0.1, 0.1, 0.1, 0.02, 0.02, 0.1, 0.1, 0.1, 0.1, 0.003, 0.003, 0.003;
    return s;
}

// Tolerances for the double build and the float (ESP32) build.
constexpr bool kFloat = sizeof(real) == 4;
constexpr double kTight = kFloat ? 1e-4 : 1e-6;

bool is_spd(const ErrMat& P) {
    if (!P.isApprox(P.transpose(), static_cast<real>(1e-6))) return false;
    Eigen::LLT<ErrMat> llt(P);
    return llt.info() == Eigen::Success;
}

}  // namespace

TEST(InsFilter, StaysPutWhenLevelAndStill) {
    InsFilter f;
    f.set_gravity(static_cast<real>(9.81));
    State x;
    f.init(x, default_sigma());
    ImuSample s;
    s.accel = Vec3(0, 0, static_cast<real>(-9.81));
    for (int k = 1; k <= 2000; ++k) {  // 10 s at 200 Hz
        s.t_us = k * 5000;
        f.propagate(s);
    }
    EXPECT_LT(f.state().p.norm(), kTight);
    EXPECT_LT(f.state().v.norm(), kTight);
    // Uncertainty grows without updates.
    EXPECT_GT(f.cov()(0, 0), 1.0);
}

TEST(InsFilter, ForwardAccelerationIntegrates) {
    InsFilter f;
    f.set_gravity(static_cast<real>(9.81));
    State x;
    x.q = quat_from_rpy(0, 0, static_cast<real>(kPi / 2));  // facing east
    f.init(x, default_sigma());
    ImuSample s;
    s.accel = Vec3(1, 0, static_cast<real>(-9.81));  // 1 m/s^2 forward
    for (int k = 1; k <= 400; ++k) {  // 2 s
        s.t_us = k * 5000;
        f.propagate(s);
    }
    EXPECT_NEAR(f.state().v.y(), 2.0, kTight);  // east
    EXPECT_NEAR(f.state().p.y(), 2.0, 1e-3);    // 0.5 a t^2
    EXPECT_NEAR(f.state().v.x(), 0.0, kTight);
}

TEST(InsFilter, CovarianceStaysSymmetricPositiveDefinite) {
    std::mt19937 rng(7);
    std::normal_distribution<double> n(0, 1);
    InsFilter f;
    State x;
    x.v = Vec3(5, 1, 0);
    f.init(x, default_sigma());
    ImuSample s;
    for (int k = 1; k <= 100000; ++k) {
        s.t_us = k * 5000;
        s.accel = Vec3(static_cast<real>(0.3 * n(rng)), static_cast<real>(0.3 * n(rng)),
                       static_cast<real>(-9.81 + 0.3 * n(rng)));
        s.gyro = Vec3(static_cast<real>(0.05 * n(rng)), static_cast<real>(0.05 * n(rng)),
                      static_cast<real>(0.05 * n(rng)));
        f.propagate(s);
        if (k % 20 == 0) {
            const State& st = f.state();
            f.update_gnss(st.p + Vec3(static_cast<real>(n(rng)), static_cast<real>(n(rng)), 0),
                          st.v, 1, static_cast<real>(1.5), static_cast<real>(0.1));
            f.update_nhc();
        }
        if (k % 10000 == 0) {
            ASSERT_TRUE(is_spd(f.cov())) << "step " << k;
        }
    }
}

TEST(InsFilter, GateRejectsJumpUntilCovarianceIsInflated) {
    InsFilter f;
    State x;
    f.init(x, default_sigma());
    const Vec3 far(100, 0, 0);
    EXPECT_FALSE(f.update_gnss(far, Vec3::Zero(), static_cast<real>(0.5), 1, static_cast<real>(0.1)));
    EXPECT_EQ(f.diag().gnss_rejected, 1u);
    EXPECT_GT(f.last_gnss_nis(), f.config().gnss_gate);

    f.inflate_pos_vel(far - f.state().p, Vec3::Zero());
    EXPECT_TRUE(f.update_gnss(far, Vec3::Zero(), static_cast<real>(0.5), 1, static_cast<real>(0.1)));
    EXPECT_NEAR(f.state().p.x(), 100.0, 1.0);
}

TEST(InsFilter, NhcJacobianMatchesFiniteDifference) {
    // d(v_body)/d(theta) = [v_body]x, checked numerically.
    const Quat q = quat_from_rpy(static_cast<real>(0.2), static_cast<real>(-0.1), static_cast<real>(1.0));
    const Vec3 v(4, -3, static_cast<real>(0.5));
    const Vec3 vb = q.toRotationMatrix().transpose() * v;
    const real e = static_cast<real>(kFloat ? 1e-3 : 1e-6);
    for (int i = 0; i < 3; ++i) {
        Vec3 d = Vec3::Zero();
        d(i) = e;
        const Quat qp = q * quat_from_rotvec(d);
        const Vec3 vbp = qp.toRotationMatrix().transpose() * v;
        const Vec3 numeric = (vbp - vb) / e;
        const Vec3 analytic = skew(vb).col(i);
        EXPECT_LT((numeric - analytic).norm(), kFloat ? 2e-2 : 1e-4) << "axis " << i;
    }
}

TEST(InsFilter, NhcPullsHeadingTowardTheVelocity) {
    // Velocity points north but the filter thinks the bike faces 10 degrees east of
    // north: the constraint should rotate the attitude toward the velocity.
    InsFilter f;
    State x;
    x.v = Vec3(6, 0, 0);
    x.q = quat_from_rpy(0, 0, static_cast<real>(10 * kDeg));
    ErrVec sig = default_sigma();
    sig(kTh + 2) = static_cast<real>(20 * kDeg);
    sig.segment<3>(kV).setConstant(static_cast<real>(0.02));
    f.init(x, sig);
    for (int i = 0; i < 20; ++i) f.update_nhc();
    const real yaw = rpy_from_quat(f.state().q).z();
    EXPECT_LT(std::abs(yaw), 2 * kDeg);
}

TEST(InsFilter, ZuptEstimatesGyroBias) {
    InsFilter f;
    State x;
    f.init(x, default_sigma());
    const Vec3 gyro(static_cast<real>(0.002), static_cast<real>(-0.001), static_cast<real>(0.0015));
    for (int i = 0; i < 50; ++i) f.update_zupt(gyro);
    EXPECT_LT((f.state().bg - gyro).norm(), 2e-4);
}
