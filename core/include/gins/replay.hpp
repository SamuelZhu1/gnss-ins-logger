// Host-only helpers behind the Python tools: replay a logged ride through the
// Navigator, and map local timestamps to GPS time with the same TimeBase the
// firmware uses.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "gins/navigator.hpp"

namespace gins {

struct ReplayInput {
    std::size_t n_imu = 0;
    const int64_t* imu_t = nullptr;
    const double* accel = nullptr;  // n_imu x 3, row major, body frame
    const double* gyro = nullptr;

    std::size_t n_fix = 0;
    const int64_t* fix_t = nullptr;
    const double* lat = nullptr;
    const double* lon = nullptr;
    const double* h = nullptr;
    const double* vel = nullptr;  // n_fix x 3, NED
    const double* hacc = nullptr;
    const double* vacc = nullptr;
    const double* sacc = nullptr;
    const uint8_t* fix_type = nullptr;
    const uint8_t* carr_soln = nullptr;

    std::size_t n_mask = 0;
    const int64_t* mask_start = nullptr;
    const int64_t* mask_end = nullptr;
};

struct ReplayOutput {
    std::vector<int64_t> t;
    std::vector<double> lat, lon, h;
    std::vector<double> vn, ve, vd;
    std::vector<double> roll, pitch, yaw;
    std::vector<double> sig_n, sig_e, sig_d;
    std::vector<uint8_t> mode;
    std::vector<uint8_t> masked;
    NavDiag nav_diag;
    FilterDiag filter_diag;
};

ReplayOutput replay(const ReplayInput& in, const NavConfig& cfg);

struct TimeMapResult {
    std::vector<int64_t> gps_us;
    double drift_ppm = 0;
    double mean_latency_ms = 0;
    uint32_t pps_count = 0;
};

// Feeds PPS edges and NAV-PVT arrivals to a TimeBase in local-time order and
// converts each query time with the fit that existed at that moment (causal,
// exactly as the firmware would).
TimeMapResult map_local_to_gps(const int64_t* pps_local, std::size_t n_pps,
                               const int64_t* pvt_arrival, const uint32_t* pvt_itow,
                               std::size_t n_pvt, const int64_t* query, std::size_t n_query,
                               int64_t nominal_latency_us);

}  // namespace gins
