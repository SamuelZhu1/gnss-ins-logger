// u-blox UBX protocol: byte-at-a-time frame parser, NAV-PVT decoder and
// frame builders for configuring the receiver (CFG-VALSET).
#pragma once

#include <cstddef>
#include <cstdint>

#include "gins/types.hpp"

namespace gins {
namespace ubx {

constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;
constexpr std::size_t kMaxPayload = 2100;  // RXM-RAWX with 64 measurements is 2064

constexpr uint8_t kClsNav = 0x01;
constexpr uint8_t kIdNavPvt = 0x07;
constexpr uint8_t kClsRxm = 0x02;
constexpr uint8_t kIdRxmRawx = 0x15;
constexpr uint8_t kClsCfg = 0x06;
constexpr uint8_t kIdCfgValset = 0x8A;
constexpr uint8_t kClsAck = 0x05;

struct NavPvt {
    uint32_t itow_ms = 0;
    uint8_t fix_type = 0;
    uint8_t flags = 0;
    uint8_t carr_soln = 0;  // flags bits 6-7
    uint8_t num_sv = 0;
    int32_t lon_e7 = 0;
    int32_t lat_e7 = 0;
    int32_t height_mm = 0;  // above ellipsoid
    uint32_t h_acc_mm = 0;
    uint32_t v_acc_mm = 0;
    int32_t vel_n_mm_s = 0;
    int32_t vel_e_mm_s = 0;
    int32_t vel_d_mm_s = 0;
    uint32_t s_acc_mm_s = 0;
};

class Parser {
public:
    // Returns true when a frame with a good checksum is complete.
    bool feed(uint8_t byte);
    uint8_t cls() const { return cls_; }
    uint8_t id() const { return id_; }
    uint16_t len() const { return len_; }
    const uint8_t* payload() const { return payload_; }
    uint32_t checksum_errors() const { return ck_errors_; }
    uint32_t oversize() const { return oversize_; }

private:
    enum class St { kSync1, kSync2, kCls, kId, kLen1, kLen2, kPayload, kCkA, kCkB };
    St st_ = St::kSync1;
    uint8_t cls_ = 0, id_ = 0;
    uint16_t len_ = 0;
    uint16_t idx_ = 0;
    uint8_t ck_a_ = 0, ck_b_ = 0, got_a_ = 0;
    uint8_t payload_[kMaxPayload];
    uint32_t ck_errors_ = 0;
    uint32_t oversize_ = 0;
};

bool decode_nav_pvt(const uint8_t* p, std::size_t len, NavPvt& out);

// Build a GnssFix from NAV-PVT at the given epoch time.
GnssFix to_fix(const NavPvt& pvt, int64_t t_gps_us);

// Write a full UBX frame (sync, header, payload, checksum). Returns its length or 0.
std::size_t build_frame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len,
                        uint8_t* out, std::size_t cap);

// CFG-VALSET items. Value size comes from the key's size field (bits 28-30).
struct ConfigItem {
    uint32_t key;
    uint32_t value;
};
// Builds a CFG-VALSET frame writing to the RAM layer.
std::size_t build_valset(const ConfigItem* items, std::size_t n, uint8_t* out, std::size_t cap);

// Configuration keys used by the firmware (u-blox F9 interface description).
namespace key {
constexpr uint32_t kRateMeas = 0x30210001;            // U2, ms between measurements
constexpr uint32_t kUart1Baud = 0x40520001;           // U4
constexpr uint32_t kUart1OutNmea = 0x10740002;        // L
constexpr uint32_t kMsgNavPvtUart1 = 0x20910007;      // U1, output rate per epoch
constexpr uint32_t kMsgRxmRawxUart1 = 0x209102a5;     // U1
constexpr uint32_t kTpTimegridTp1 = 0x2005000c;       // E1, 1 = GPS time
}  // namespace key

}  // namespace ubx
}  // namespace gins
