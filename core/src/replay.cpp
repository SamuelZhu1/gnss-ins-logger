#include "gins/replay.hpp"

#include <algorithm>
#include <memory>
#include <numeric>

#include "gins/time_base.hpp"

namespace gins {
namespace {

void collect(const NavOutput& o, void* ctx) {
    auto* out = static_cast<ReplayOutput*>(ctx);
    out->t.push_back(o.t_us);
    out->lat.push_back(o.lla.lat_deg);
    out->lon.push_back(o.lla.lon_deg);
    out->h.push_back(o.lla.h_m);
    out->vn.push_back(o.v_ned.x());
    out->ve.push_back(o.v_ned.y());
    out->vd.push_back(o.v_ned.z());
    out->roll.push_back(o.rpy.x());
    out->pitch.push_back(o.rpy.y());
    out->yaw.push_back(o.rpy.z());
    out->sig_n.push_back(o.sigma_pos.x());
    out->sig_e.push_back(o.sigma_pos.y());
    out->sig_d.push_back(o.sigma_pos.z());
    out->mode.push_back(static_cast<uint8_t>(o.mode));
    out->masked.push_back(o.gnss_masked ? 1 : 0);
}

}  // namespace

ReplayOutput replay(const ReplayInput& in, const NavConfig& cfg) {
    ReplayOutput out;
    const std::size_t est = in.n_imu ? (in.n_imu * 5000 / static_cast<std::size_t>(std::max<int64_t>(cfg.output_period_us, 1))) + 16 : 16;
    out.t.reserve(est);

    auto nav = std::make_unique<Navigator>(cfg);
    nav->set_output(collect, &out);
    for (std::size_t i = 0; i < in.n_mask; ++i) {
        nav->add_gnss_mask(in.mask_start[i], in.mask_end[i]);
    }

    std::size_t i = 0, j = 0;
    while (i < in.n_imu || j < in.n_fix) {
        const bool take_fix = j < in.n_fix && (i >= in.n_imu || in.fix_t[j] <= in.imu_t[i]);
        if (take_fix) {
            GnssFix f;
            f.t_us = in.fix_t[j];
            f.lat_deg = in.lat[j];
            f.lon_deg = in.lon[j];
            f.h_m = in.h[j];
            f.vel_ned = Vec3(static_cast<real>(in.vel[3 * j]), static_cast<real>(in.vel[3 * j + 1]),
                             static_cast<real>(in.vel[3 * j + 2]));
            f.h_acc = static_cast<real>(in.hacc[j]);
            f.v_acc = static_cast<real>(in.vacc[j]);
            f.s_acc = static_cast<real>(in.sacc[j]);
            f.fix_type = in.fix_type[j];
            f.carr_soln = in.carr_soln ? in.carr_soln[j] : 0;
            nav->push_fix(f);
            ++j;
        } else {
            ImuSample s;
            s.t_us = in.imu_t[i];
            s.accel = Vec3(static_cast<real>(in.accel[3 * i]), static_cast<real>(in.accel[3 * i + 1]),
                           static_cast<real>(in.accel[3 * i + 2]));
            s.gyro = Vec3(static_cast<real>(in.gyro[3 * i]), static_cast<real>(in.gyro[3 * i + 1]),
                          static_cast<real>(in.gyro[3 * i + 2]));
            nav->push_imu(s);
            nav->process();
            ++i;
        }
    }
    nav->flush();
    out.nav_diag = nav->diag();
    out.filter_diag = nav->filter().diag();
    return out;
}

TimeMapResult map_local_to_gps(const int64_t* pps_local, std::size_t n_pps,
                               const int64_t* pvt_arrival, const uint32_t* pvt_itow,
                               std::size_t n_pvt, const int64_t* query, std::size_t n_query,
                               int64_t nominal_latency_us) {
    TimeMapResult r;
    r.gps_us.resize(n_query);
    TimeBase tb(nominal_latency_us);

    // Query order by local time, so results can be written back in input order.
    std::vector<std::size_t> order(n_query);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(),
                     [&](std::size_t a, std::size_t b) { return query[a] < query[b]; });

    std::size_t ip = 0, iv = 0, iq = 0;
    // Queries before the first NAV-PVT have no time reference; hold them until
    // the first message so they get the coarse offset rather than garbage.
    bool started = false;
    std::vector<std::size_t> held;
    while (iq < n_query) {
        const int64_t tp = ip < n_pps ? pps_local[ip] : INT64_MAX;
        const int64_t tv = iv < n_pvt ? pvt_arrival[iv] : INT64_MAX;
        const int64_t tq = query[order[iq]];
        if (tp <= tv && tp <= tq) {
            tb.on_pps(tp);
            ++ip;
        } else if (tv <= tq) {
            tb.on_pvt(tv, pvt_itow[iv]);
            ++iv;
            if (!started) {
                started = true;
                for (std::size_t k : held) r.gps_us[k] = tb.to_gps_us(query[k]);
                held.clear();
            }
        } else {
            if (started) {
                r.gps_us[order[iq]] = tb.to_gps_us(tq);
            } else {
                held.push_back(order[iq]);
            }
            ++iq;
        }
    }
    for (std::size_t k : held) r.gps_us[k] = tb.to_gps_us(query[k]);
    // Feed what's left so the drift and latency statistics cover the whole log.
    while (ip < n_pps || iv < n_pvt) {
        const int64_t tp = ip < n_pps ? pps_local[ip] : INT64_MAX;
        const int64_t tv = iv < n_pvt ? pvt_arrival[iv] : INT64_MAX;
        if (tp <= tv) {
            tb.on_pps(tp);
            ++ip;
        } else {
            tb.on_pvt(tv, pvt_itow[iv]);
            ++iv;
        }
    }
    r.drift_ppm = tb.drift_ppm();
    r.mean_latency_ms = tb.mean_latency_ms();
    r.pps_count = tb.pps_count();
    return r;
}

}  // namespace gins
