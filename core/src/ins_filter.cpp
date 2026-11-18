#include "gins/ins_filter.hpp"

#include <Eigen/Cholesky>

#include "gins/rotation.hpp"

namespace gins {

InsFilter::InsFilter(const FilterConfig& cfg) : cfg_(cfg) {}

void InsFilter::init(const State& s, const ErrVec& sigma0) {
    x_ = s;
    x_.q.normalize();
    P_ = sigma0.cwiseProduct(sigma0).asDiagonal();
    diag_ = FilterDiag{};
}

void InsFilter::propagate(const ImuSample& imu) {
    propagate_to(imu.t_us, imu.accel, imu.gyro);
}

void InsFilter::propagate_to(int64_t t_us, const Vec3& accel, const Vec3& gyro) {
    if (t_us <= x_.t_us) {
        return;
    }
    real dt = static_cast<real>(t_us - x_.t_us) * static_cast<real>(1e-6);
    if (dt > cfg_.max_dt) {
        // A gap in the IMU stream. Integrating a long step with one sample would
        // be wrong either way; clamp it and count it so it shows up in the logs.
        ++diag_.imu_gaps;
        dt = cfg_.max_dt;
    }
    x_.t_us = t_us;

    const Vec3 w = gyro - x_.bg;
    const Vec3 f = accel - x_.ba;
    const Mat3 R = x_.q.toRotationMatrix();
    const Vec3 a = R * f + Vec3(0, 0, gravity_);

    // Nominal state (strapdown mechanization in local NED).
    x_.p += x_.v * dt + a * (static_cast<real>(0.5) * dt * dt);
    x_.v += a * dt;
    x_.q = (x_.q * quat_from_rotvec(w * dt)).normalized();

    // Error state transition, first order.
    ErrMat F = ErrMat::Identity();
    F.block<3, 3>(kP, kV) = Mat3::Identity() * dt;
    F.block<3, 3>(kV, kTh) = -R * skew(f) * dt;
    F.block<3, 3>(kV, kBa) = -R * dt;
    F.block<3, 3>(kTh, kTh) = quat_from_rotvec(w * dt).toRotationMatrix().transpose();
    F.block<3, 3>(kTh, kBg) = -Mat3::Identity() * dt;

    ErrVec q = ErrVec::Zero();
    q.segment<3>(kV).setConstant(cfg_.accel_noise * cfg_.accel_noise * dt);
    q.segment<3>(kTh).setConstant(cfg_.gyro_noise * cfg_.gyro_noise * dt);
    q.segment<3>(kBa).setConstant(cfg_.accel_bias_rw * cfg_.accel_bias_rw * dt);
    q.segment<3>(kBg).setConstant(cfg_.gyro_bias_rw * cfg_.gyro_bias_rw * dt);

    P_ = F * P_ * F.transpose();
    P_.diagonal() += q;
    symmetrize();
}

template <int M>
bool InsFilter::kalman_update(const Eigen::Matrix<real, M, 1>& y,
                              const Eigen::Matrix<real, M, kErr>& H,
                              const Eigen::Matrix<real, M, M>& R, real gate, real* nis_out) {
    using MatM = Eigen::Matrix<real, M, M>;
    const Eigen::Matrix<real, kErr, M> PHt = P_ * H.transpose();
    const MatM S = H * PHt + R;
    const Eigen::LLT<MatM> llt(S);
    if (llt.info() != Eigen::Success) {
        return false;
    }
    const real nis = y.dot(llt.solve(y));
    if (nis_out) {
        *nis_out = nis;
    }
    if (nis > gate) {
        return false;
    }
    // K = P H^T S^-1, computed as (S^-1 H P)^T since S and P are symmetric.
    const Eigen::Matrix<real, kErr, M> K = llt.solve(PHt.transpose()).transpose();
    const ErrVec dx = K * y;

    // Joseph form keeps P symmetric positive definite, which matters in float.
    const ErrMat IKH = ErrMat::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    symmetrize();
    inject(dx);
    return true;
}

void InsFilter::inject(const ErrVec& dx) {
    x_.p += dx.segment<3>(kP);
    x_.v += dx.segment<3>(kV);
    const Vec3 dth = dx.segment<3>(kTh);
    x_.q = (x_.q * quat_from_rotvec(dth)).normalized();
    x_.ba += dx.segment<3>(kBa);
    x_.bg += dx.segment<3>(kBg);

    // Reset the attitude error: P = G P G^T with G = I - [dtheta/2]x on that block.
    ErrMat G = ErrMat::Identity();
    G.block<3, 3>(kTh, kTh) = Mat3::Identity() - skew(dth * static_cast<real>(0.5));
    P_ = G * P_ * G.transpose();
    symmetrize();
}

void InsFilter::symmetrize() {
    P_ = (P_ + P_.transpose()) * static_cast<real>(0.5);
    // Keep a small floor on the diagonal so float rounding can't push a variance negative.
    for (int i = 0; i < kErr; ++i) {
        if (P_(i, i) < static_cast<real>(1e-12)) {
            P_(i, i) = static_cast<real>(1e-12);
        }
    }
}

bool InsFilter::update_gnss(const Vec3& p_ned, const Vec3& v_ned, real h_acc, real v_acc,
                            real s_acc, bool force) {
    Eigen::Matrix<real, 6, 1> y;
    y.segment<3>(0) = p_ned - x_.p;
    y.segment<3>(3) = v_ned - x_.v;

    Eigen::Matrix<real, 6, kErr> H = Eigen::Matrix<real, 6, kErr>::Zero();
    H.block<3, 3>(0, kP).setIdentity();
    H.block<3, 3>(3, kV).setIdentity();

    const real k = cfg_.gnss_r_scale;
    Eigen::Matrix<real, 6, 1> r;
    r << h_acc * h_acc, h_acc * h_acc, v_acc * v_acc, s_acc * s_acc, s_acc * s_acc, s_acc * s_acc;
    const Eigen::Matrix<real, 6, 6> R = (r * (k * k)).asDiagonal();

    const real gate = force ? static_cast<real>(1e30) : cfg_.gnss_gate;
    real nis = 0;
    const bool ok = kalman_update<6>(y, H, R, gate, &nis);
    diag_.last_gnss_nis = nis;
    if (ok) {
        ++diag_.gnss_accepted;
        if (force) {
            ++diag_.gnss_forced;
        }
    } else {
        ++diag_.gnss_rejected;
    }
    return ok;
}

bool InsFilter::update_nhc() {
    if (!cfg_.use_nhc || x_.v.norm() < cfg_.nhc_min_speed) {
        return false;
    }
    const Mat3 Rt = x_.q.toRotationMatrix().transpose();
    const Vec3 vb = Rt * x_.v;

    // v_body = R^T v; perturbing gives d(v_body) = R^T dv + [v_body]x dtheta.
    Eigen::Matrix<real, 2, kErr> H = Eigen::Matrix<real, 2, kErr>::Zero();
    H.block<2, 3>(0, kV) = Rt.block<2, 3>(1, 0);
    H.block<2, 3>(0, kTh) = skew(vb).block<2, 3>(1, 0);

    const Eigen::Matrix<real, 2, 1> y = -vb.segment<2>(1);
    const Eigen::Matrix<real, 2, 2> R =
        Eigen::Matrix<real, 2, 2>::Identity() * (cfg_.nhc_sigma * cfg_.nhc_sigma);
    const bool ok = kalman_update<2>(y, H, R, static_cast<real>(1e30), nullptr);
    if (ok) {
        ++diag_.nhc_updates;
    }
    return ok;
}

bool InsFilter::update_zupt(const Vec3& gyro_meas) {
    if (!cfg_.use_zupt) {
        return false;
    }
    // Rows 0-2: velocity is zero. Rows 3-5: true angular rate is zero, so the
    // gyro reading is the bias.
    Eigen::Matrix<real, 6, 1> y;
    y.segment<3>(0) = -x_.v;
    y.segment<3>(3) = gyro_meas - x_.bg;

    Eigen::Matrix<real, 6, kErr> H = Eigen::Matrix<real, 6, kErr>::Zero();
    H.block<3, 3>(0, kV).setIdentity();
    H.block<3, 3>(3, kBg).setIdentity();

    Eigen::Matrix<real, 6, 1> r;
    const real zv = cfg_.zupt_sigma * cfg_.zupt_sigma;
    const real zw = cfg_.zaru_sigma * cfg_.zaru_sigma;
    r << zv, zv, zv, zw, zw, zw;
    const Eigen::Matrix<real, 6, 6> R = r.asDiagonal();
    const bool ok = kalman_update<6>(y, H, R, static_cast<real>(1e30), nullptr);
    if (ok) {
        ++diag_.zupt_updates;
    }
    return ok;
}

void InsFilter::inflate_pos_vel(const Vec3& dp, const Vec3& dv) {
    for (int i = 0; i < 3; ++i) {
        P_(kP + i, kP + i) += dp(i) * dp(i);
        P_(kV + i, kV + i) += dv(i) * dv(i);
    }
}

const char* mode_name(Mode m) {
    switch (m) {
        case Mode::kWaitFix: return "WAIT_FIX";
        case Mode::kAligning: return "ALIGNING";
        case Mode::kWaitYaw: return "WAIT_YAW";
        case Mode::kNav: return "NAV";
        case Mode::kDegraded: return "DEGRADED";
    }
    return "?";
}

}  // namespace gins
