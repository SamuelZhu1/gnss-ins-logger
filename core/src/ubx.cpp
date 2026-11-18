#include "gins/ubx.hpp"

#include <cstring>

namespace gins {
namespace ubx {
namespace {

uint32_t rd_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
int32_t rd_i32(const uint8_t* p) { return static_cast<int32_t>(rd_u32(p)); }

void checksum(const uint8_t* p, std::size_t n, uint8_t& a, uint8_t& b) {
    a = 0;
    b = 0;
    for (std::size_t i = 0; i < n; ++i) {
        a = static_cast<uint8_t>(a + p[i]);
        b = static_cast<uint8_t>(b + a);
    }
}

}  // namespace

bool Parser::feed(uint8_t c) {
    switch (st_) {
        case St::kSync1:
            if (c == kSync1) st_ = St::kSync2;
            return false;
        case St::kSync2:
            st_ = (c == kSync2) ? St::kCls : (c == kSync1 ? St::kSync2 : St::kSync1);
            return false;
        case St::kCls:
            cls_ = c;
            ck_a_ = c;
            ck_b_ = c;
            st_ = St::kId;
            return false;
        case St::kId:
            id_ = c;
            ck_a_ = static_cast<uint8_t>(ck_a_ + c);
            ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_);
            st_ = St::kLen1;
            return false;
        case St::kLen1:
            len_ = c;
            ck_a_ = static_cast<uint8_t>(ck_a_ + c);
            ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_);
            st_ = St::kLen2;
            return false;
        case St::kLen2:
            len_ = static_cast<uint16_t>(len_ | (c << 8));
            ck_a_ = static_cast<uint8_t>(ck_a_ + c);
            ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_);
            if (len_ > kMaxPayload) {
                ++oversize_;
                st_ = St::kSync1;
                return false;
            }
            idx_ = 0;
            st_ = len_ ? St::kPayload : St::kCkA;
            return false;
        case St::kPayload:
            payload_[idx_++] = c;
            ck_a_ = static_cast<uint8_t>(ck_a_ + c);
            ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_);
            if (idx_ == len_) st_ = St::kCkA;
            return false;
        case St::kCkA:
            got_a_ = c;
            st_ = St::kCkB;
            return false;
        case St::kCkB:
            st_ = St::kSync1;
            if (got_a_ == ck_a_ && c == ck_b_) {
                return true;
            }
            ++ck_errors_;
            return false;
    }
    return false;
}

bool decode_nav_pvt(const uint8_t* p, std::size_t len, NavPvt& o) {
    if (len < 92) {
        return false;
    }
    o.itow_ms = rd_u32(p + 0);
    o.fix_type = p[20];
    o.flags = p[21];
    o.carr_soln = static_cast<uint8_t>((p[21] >> 6) & 0x03);
    o.num_sv = p[23];
    o.lon_e7 = rd_i32(p + 24);
    o.lat_e7 = rd_i32(p + 28);
    o.height_mm = rd_i32(p + 32);
    o.h_acc_mm = rd_u32(p + 40);
    o.v_acc_mm = rd_u32(p + 44);
    o.vel_n_mm_s = rd_i32(p + 48);
    o.vel_e_mm_s = rd_i32(p + 52);
    o.vel_d_mm_s = rd_i32(p + 56);
    o.s_acc_mm_s = rd_u32(p + 68);
    return true;
}

GnssFix to_fix(const NavPvt& pvt, int64_t t_gps_us) {
    GnssFix f;
    f.t_us = t_gps_us;
    f.lat_deg = pvt.lat_e7 * 1e-7;
    f.lon_deg = pvt.lon_e7 * 1e-7;
    f.h_m = pvt.height_mm * 1e-3;
    f.vel_ned = Vec3(static_cast<real>(pvt.vel_n_mm_s * 1e-3), static_cast<real>(pvt.vel_e_mm_s * 1e-3),
                     static_cast<real>(pvt.vel_d_mm_s * 1e-3));
    f.h_acc = static_cast<real>(pvt.h_acc_mm * 1e-3);
    f.v_acc = static_cast<real>(pvt.v_acc_mm * 1e-3);
    f.s_acc = static_cast<real>(pvt.s_acc_mm_s * 1e-3);
    // gnssFixOK (flags bit 0) clear means the receiver itself doesn't trust it.
    f.fix_type = (pvt.flags & 0x01) ? pvt.fix_type : 0;
    f.carr_soln = pvt.carr_soln;
    f.num_sv = pvt.num_sv;
    return f;
}

std::size_t build_frame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len,
                        uint8_t* out, std::size_t cap) {
    const std::size_t total = 8u + len;
    if (total > cap) {
        return 0;
    }
    out[0] = kSync1;
    out[1] = kSync2;
    out[2] = cls;
    out[3] = id;
    out[4] = static_cast<uint8_t>(len & 0xFF);
    out[5] = static_cast<uint8_t>(len >> 8);
    if (len) {
        std::memcpy(out + 6, payload, len);
    }
    uint8_t a, b;
    checksum(out + 2, 4u + len, a, b);
    out[6 + len] = a;
    out[7 + len] = b;
    return total;
}

std::size_t build_valset(const ConfigItem* items, std::size_t n, uint8_t* out, std::size_t cap) {
    uint8_t payload[4 + 64 * 8];
    std::size_t k = 0;
    payload[k++] = 0x00;  // version
    payload[k++] = 0x01;  // layers: RAM
    payload[k++] = 0x00;
    payload[k++] = 0x00;
    for (std::size_t i = 0; i < n && i < 64; ++i) {
        const uint32_t key = items[i].key;
        for (int b = 0; b < 4; ++b) payload[k++] = static_cast<uint8_t>((key >> (8 * b)) & 0xFF);
        const uint32_t size_code = (key >> 28) & 0x07;
        const int nbytes = size_code == 1 ? 1 : size_code == 2 ? 1 : size_code == 3 ? 2 : 4;
        for (int b = 0; b < nbytes; ++b) payload[k++] = static_cast<uint8_t>((items[i].value >> (8 * b)) & 0xFF);
    }
    return build_frame(kClsCfg, kIdCfgValset, payload, static_cast<uint16_t>(k), out, cap);
}

}  // namespace ubx
}  // namespace gins
