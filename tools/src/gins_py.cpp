// pybind11 bindings for the replay, time mapping and log helpers.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "gins/geo.hpp"
#include "gins/log_format.hpp"
#include "gins/replay.hpp"

namespace py = pybind11;
using gins::NavConfig;

namespace {

template <typename T>
using arr = py::array_t<T, py::array::c_style | py::array::forcecast>;

template <typename T>
py::array_t<T> to_np(const std::vector<T>& v) {
    py::array_t<T> a(static_cast<py::ssize_t>(v.size()));
    if (!v.empty()) std::memcpy(a.mutable_data(), v.data(), v.size() * sizeof(T));
    return a;
}

NavConfig config_from(const py::dict& d) {
    NavConfig c;
    auto get = [&](const char* k, auto& field) {
        if (d.contains(k)) field = d[k].cast<std::decay_t<decltype(field)>>();
    };
    get("lag_us", c.lag_us);
    get("align_us", c.align_us);
    get("origin_max_hacc", c.origin_max_hacc);
    get("max_hacc", c.max_hacc);
    get("yaw_speed", c.yaw_speed);
    get("yaw_hold_us", c.yaw_hold_us);
    get("degraded_after_us", c.degraded_after_us);
    get("max_consecutive_rejects", c.max_consecutive_rejects);
    get("output_period_us", c.output_period_us);
    get("aux_period_us", c.aux_period_us);
    get("still_accel_std", c.still_accel_std);
    get("still_gyro", c.still_gyro);
    get("accel_noise", c.filter.accel_noise);
    get("gyro_noise", c.filter.gyro_noise);
    get("accel_bias_rw", c.filter.accel_bias_rw);
    get("gyro_bias_rw", c.filter.gyro_bias_rw);
    get("gnss_gate", c.filter.gnss_gate);
    get("gnss_r_scale", c.filter.gnss_r_scale);
    get("use_nhc", c.filter.use_nhc);
    get("nhc_sigma", c.filter.nhc_sigma);
    get("nhc_min_speed", c.filter.nhc_min_speed);
    get("use_zupt", c.filter.use_zupt);
    get("zupt_sigma", c.filter.zupt_sigma);
    get("zaru_sigma", c.filter.zaru_sigma);
    return c;
}

py::dict replay(arr<int64_t> imu_t, arr<double> accel, arr<double> gyro, arr<int64_t> fix_t,
                arr<double> lat, arr<double> lon, arr<double> h, arr<double> vel, arr<double> hacc,
                arr<double> vacc, arr<double> sacc, arr<uint8_t> fix_type, arr<uint8_t> carr,
                arr<int64_t> mask_start, arr<int64_t> mask_end, py::dict config) {
    const auto n = static_cast<std::size_t>(imu_t.size());
    const auto m = static_cast<std::size_t>(fix_t.size());
    if (static_cast<std::size_t>(accel.size()) != 3 * n || static_cast<std::size_t>(gyro.size()) != 3 * n)
        throw std::invalid_argument("accel and gyro must be (N, 3)");
    if (static_cast<std::size_t>(vel.size()) != 3 * m) throw std::invalid_argument("vel must be (M, 3)");
    if (mask_start.size() > static_cast<py::ssize_t>(gins::Navigator::kMaxMasks))
        throw std::invalid_argument("at most 64 masks per replay");

    gins::ReplayInput in;
    in.n_imu = n;
    in.imu_t = imu_t.data();
    in.accel = accel.data();
    in.gyro = gyro.data();
    in.n_fix = m;
    in.fix_t = fix_t.data();
    in.lat = lat.data();
    in.lon = lon.data();
    in.h = h.data();
    in.vel = vel.data();
    in.hacc = hacc.data();
    in.vacc = vacc.data();
    in.sacc = sacc.data();
    in.fix_type = fix_type.data();
    in.carr_soln = carr.data();
    in.n_mask = static_cast<std::size_t>(mask_start.size());
    in.mask_start = mask_start.data();
    in.mask_end = mask_end.data();

    const NavConfig cfg = config_from(config);
    gins::ReplayOutput out;
    {
        py::gil_scoped_release release;
        out = gins::replay(in, cfg);
    }
    py::dict d;
    d["t_gps_us"] = to_np(out.t);
    d["lat"] = to_np(out.lat);
    d["lon"] = to_np(out.lon);
    d["h"] = to_np(out.h);
    d["vn"] = to_np(out.vn);
    d["ve"] = to_np(out.ve);
    d["vd"] = to_np(out.vd);
    d["roll"] = to_np(out.roll);
    d["pitch"] = to_np(out.pitch);
    d["yaw"] = to_np(out.yaw);
    d["sig_n"] = to_np(out.sig_n);
    d["sig_e"] = to_np(out.sig_e);
    d["sig_d"] = to_np(out.sig_d);
    d["mode"] = to_np(out.mode);
    d["masked"] = to_np(out.masked);
    py::dict diag;
    diag["gnss_accepted"] = out.filter_diag.gnss_accepted;
    diag["gnss_rejected"] = out.filter_diag.gnss_rejected;
    diag["gnss_forced"] = out.filter_diag.gnss_forced;
    diag["nhc_updates"] = out.filter_diag.nhc_updates;
    diag["zupt_updates"] = out.filter_diag.zupt_updates;
    diag["imu_gaps"] = out.filter_diag.imu_gaps;
    diag["fixes_masked"] = out.nav_diag.fixes_masked;
    diag["fixes_ignored"] = out.nav_diag.fixes_ignored;
    diag["fixes_late"] = out.nav_diag.fixes_late;
    diag["coarse_alignments"] = out.nav_diag.coarse_alignments;
    diag["forced_resets"] = out.nav_diag.forced_resets;
    diag["align_restarts"] = out.nav_diag.align_restarts;
    d["diag"] = diag;
    return d;
}

py::tuple map_local_to_gps(arr<int64_t> pps, arr<int64_t> pvt_arrival, arr<uint32_t> pvt_itow,
                           arr<int64_t> query, int64_t nominal_latency_us) {
    if (pvt_arrival.size() != pvt_itow.size()) throw std::invalid_argument("pvt arrays differ in length");
    gins::TimeMapResult r;
    {
        py::gil_scoped_release release;
        r = gins::map_local_to_gps(pps.data(), static_cast<std::size_t>(pps.size()), pvt_arrival.data(),
                                   pvt_itow.data(), static_cast<std::size_t>(pvt_itow.size()), query.data(),
                                   static_cast<std::size_t>(query.size()), nominal_latency_us);
    }
    return py::make_tuple(to_np(r.gps_us), r.drift_ppm, r.mean_latency_ms, r.pps_count);
}

// Split a raw log into per-type record lists: seq numbers and payload bytes.
py::dict split_records(py::bytes data) {
    std::string buf = data;
    struct Acc {
        std::vector<uint32_t> seq;
        std::vector<uint16_t> len;
        std::string payload;
    };
    std::map<int, Acc> acc;
    gins::logfmt::Reader reader;
    auto on_record = [&](const gins::logfmt::Reader::Record& r) {
        Acc& a = acc[static_cast<int>(r.type)];
        a.seq.push_back(r.seq);
        a.len.push_back(r.len);
        a.payload.append(reinterpret_cast<const char*>(r.payload), r.len);
    };
    {
        py::gil_scoped_release release;
        reader.feed(reinterpret_cast<const uint8_t*>(buf.data()), buf.size(), on_record);
        reader.finish(on_record);
    }
    py::dict out;
    for (auto& kv : acc) {
        py::dict t;
        t["seq"] = to_np(kv.second.seq);
        t["len"] = to_np(kv.second.len);
        t["payload"] = py::bytes(kv.second.payload);
        out[py::int_(kv.first)] = t;
    }
    out["crc_errors"] = reader.crc_errors();
    out["resyncs"] = reader.resyncs();
    return out;
}

py::bytes encode_record(int type, uint32_t seq, py::bytes payload) {
    std::string p = payload;
    std::string out(p.size() + gins::logfmt::kOverhead, '\0');
    const std::size_t n = gins::logfmt::encode(static_cast<gins::logfmt::Type>(type), seq,
                                               reinterpret_cast<const uint8_t*>(p.data()),
                                               static_cast<uint16_t>(p.size()),
                                               reinterpret_cast<uint8_t*>(out.data()), out.size());
    if (n == 0) throw std::invalid_argument("payload too long for this record type");
    out.resize(n);
    return py::bytes(out);
}

py::array_t<double> lla_to_ned(double lat0, double lon0, double h0, arr<double> lat, arr<double> lon,
                               arr<double> h) {
    const gins::LocalFrame f({lat0, lon0, h0});
    const auto n = lat.size();
    py::array_t<double> out({n, static_cast<py::ssize_t>(3)});
    auto o = out.mutable_unchecked<2>();
    for (py::ssize_t i = 0; i < n; ++i) {
        const auto v = f.to_ned({lat.data()[i], lon.data()[i], h.data()[i]});
        o(i, 0) = v.x();
        o(i, 1) = v.y();
        o(i, 2) = v.z();
    }
    return out;
}

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "C++ core of the GNSS/INS logger (filter, navigator, time base, log format)";
    m.def("replay", &replay, "Replay one ride through the navigator");
    m.def("map_local_to_gps", &map_local_to_gps, "Map local timestamps to GPS time with the PPS time base");
    m.def("split_records", &split_records, "Split a raw log into per-type records");
    m.def("encode_record", &encode_record, "Encode one log record");
    m.def("crc16", [](py::bytes b) {
        std::string s = b;
        return gins::logfmt::crc16(reinterpret_cast<const uint8_t*>(s.data()), s.size());
    });
    m.def("lla_to_ned", &lla_to_ned, "Convert arrays of lat/lon/h to NED around an origin");
    m.attr("MAX_MASKS") = gins::Navigator::kMaxMasks;
}
