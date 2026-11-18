// On-board log format. Every record:
//   sync 0xA5 | type u8 | len u16 | seq u32 | payload[len] | crc16 (CCITT-FALSE over type..payload)
// All fields little-endian. seq counts per record type, so a gap means records
// of that type were dropped before they reached the card.
//
// Payload layouts (Python struct notation, mirrored in tools/gins/logfmt.py):
//   IMU      0x01 '<q7h'           t_local_us, ax ay az gx gy gz (raw counts, sensor frame), temp raw
//   GNSS_PVT 0x02 '<qI3i3i3I4B'    t_local_arrival_us, itow_ms, lat_e7 lon_e7 h_mm,
//                                  vn ve vd (mm/s), hacc vacc (mm) sacc (mm/s),
//                                  fix_type carr_soln num_sv reserved
//   PPS      0x03 '<q'             t_local_us of the rising edge
//   FILTER   0x04 '<qdd10fB3x'     t_gps_us, lat lon (deg), h, vn ve vd, roll pitch yaw (rad),
//                                  sig_n sig_e sig_d, mode
//   RAWX     0x05 '<q' + bytes     t_local_arrival_us, then the UBX RXM-RAWX payload
//   EVENT    0x06 '<qHHI'          t_local_us, code, reserved, value
//   HEADER   0x10 '<16s4f3fII'     firmware version, accel scale (m/s^2 per count),
//                                  gyro scale (rad/s per count), temp scale, temp offset,
//                                  mount roll pitch yaw (deg), imu rate (Hz), config hash
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace gins {
namespace logfmt {

constexpr uint8_t kSync = 0xA5;
constexpr std::size_t kHeaderLen = 8;
constexpr std::size_t kOverhead = kHeaderLen + 2;
constexpr std::size_t kMaxPayload = 2200;

enum class Type : uint8_t {
    kImu = 0x01,
    kGnssPvt = 0x02,
    kPps = 0x03,
    kFilter = 0x04,
    kRawx = 0x05,
    kEvent = 0x06,
    kHeader = 0x10,
};

enum class Event : uint16_t {
    kSdStallMs = 1,
    kRingHighWater = 2,
    kDroppedImu = 3,
    kDroppedRawx = 4,
    kFilterOverrun = 5,
    kMissedImuIrq = 6,
    kModeChange = 7,
    kGnssMaskStart = 8,
    kLogStart = 9,
};

uint16_t crc16(const uint8_t* data, std::size_t len, uint16_t crc = 0xFFFF);

// Largest payload a record of this type can have, or 0 for an unknown type.
// The reader uses this to reject false sync bytes quickly instead of waiting
// for a long record that will never arrive.
std::size_t max_payload(uint8_t type);

// Writes one record into out. Returns the record length, or 0 if it doesn't fit.
std::size_t encode(Type type, uint32_t seq, const uint8_t* payload, uint16_t len,
                   uint8_t* out, std::size_t cap);

// Little-endian payload builder.
class Writer {
public:
    Writer(uint8_t* buf, std::size_t cap) : buf_(buf), cap_(cap) {}
    Writer& u8(uint8_t v);
    Writer& u16(uint16_t v);
    Writer& u32(uint32_t v);
    Writer& i16(int16_t v) { return u16(static_cast<uint16_t>(v)); }
    Writer& i32(int32_t v) { return u32(static_cast<uint32_t>(v)); }
    Writer& i64(int64_t v);
    Writer& f32(float v);
    Writer& f64(double v);
    Writer& bytes(const uint8_t* p, std::size_t n);
    Writer& zeros(std::size_t n);
    std::size_t size() const { return n_; }
    bool ok() const { return ok_; }

private:
    uint8_t* buf_;
    std::size_t cap_;
    std::size_t n_ = 0;
    bool ok_ = true;
};

// Incremental reader: feed bytes in any chunk size; on_record(const Record&) is
// called for every record with a good CRC. Bad CRCs are counted and the reader
// resyncs on the next sync byte, so one corrupt record costs only itself.
class Reader {
public:
    struct Record {
        Type type;
        uint32_t seq;
        const uint8_t* payload;
        uint16_t len;
    };

    template <typename F>
    void feed(const uint8_t* data, std::size_t n, F&& on_record) {
        while (n > 0) {
            const std::size_t room = sizeof(buf_) - n_;
            const std::size_t take = n < room ? n : room;
            std::memcpy(buf_ + n_, data, take);
            n_ += take;
            data += take;
            n -= take;
            parse(on_record);
        }
    }
    // End of stream: bytes stuck behind a false header get rescanned.
    template <typename F>
    void finish(F&& on_record) {
        while (n_ > 0) {
            std::memmove(buf_, buf_ + 1, n_ - 1);
            --n_;
            ++resyncs_;
            parse(on_record);
        }
    }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t resyncs() const { return resyncs_; }

private:
    template <typename F>
    void parse(F& on_record) {
        std::size_t pos = 0;
        while (n_ - pos >= kHeaderLen) {
            if (buf_[pos] != kSync) {
                ++pos;
                ++skipped_;
                continue;
            }
            const uint16_t len = static_cast<uint16_t>(buf_[pos + 2] | (buf_[pos + 3] << 8));
            if (len > max_payload(buf_[pos + 1])) {
                ++resyncs_;
                ++pos;
                continue;
            }
            const std::size_t total = kOverhead + len;
            if (n_ - pos < total) {
                break;
            }
            const uint8_t* r = buf_ + pos;
            const uint16_t got = static_cast<uint16_t>(r[kHeaderLen + len] | (r[kHeaderLen + len + 1] << 8));
            if (got != crc16(r + 1, kHeaderLen - 1 + len)) {
                ++crc_errors_;
                ++pos;  // drop this false sync byte and look for the next one
                continue;
            }
            Record rec;
            rec.type = static_cast<Type>(r[1]);
            rec.seq = static_cast<uint32_t>(r[4]) | (static_cast<uint32_t>(r[5]) << 8) |
                      (static_cast<uint32_t>(r[6]) << 16) | (static_cast<uint32_t>(r[7]) << 24);
            rec.payload = r + kHeaderLen;
            rec.len = len;
            on_record(rec);
            pos += total;
        }
        if (pos > 0) {
            std::memmove(buf_, buf_ + pos, n_ - pos);
            n_ -= pos;
        }
    }

    uint8_t buf_[2 * (kMaxPayload + kOverhead)];
    std::size_t n_ = 0;
    uint32_t crc_errors_ = 0;
    uint32_t resyncs_ = 0;
    uint32_t skipped_ = 0;
};

}  // namespace logfmt
}  // namespace gins
