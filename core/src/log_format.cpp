#include "gins/log_format.hpp"

#include <cstring>

namespace gins {
namespace logfmt {

uint16_t crc16(const uint8_t* data, std::size_t len, uint16_t crc) {
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::size_t max_payload(uint8_t type) {
    switch (static_cast<Type>(type)) {
        case Type::kImu: return 22;
        case Type::kGnssPvt: return 52;
        case Type::kPps: return 8;
        case Type::kFilter: return 68;
        case Type::kRawx: return kMaxPayload;
        case Type::kEvent: return 16;
        case Type::kHeader: return 64;
    }
    return 0;
}

std::size_t encode(Type type, uint32_t seq, const uint8_t* payload, uint16_t len,
                   uint8_t* out, std::size_t cap) {
    const std::size_t total = kOverhead + len;
    if (total > cap || len > max_payload(static_cast<uint8_t>(type))) {
        return 0;
    }
    out[0] = kSync;
    out[1] = static_cast<uint8_t>(type);
    out[2] = static_cast<uint8_t>(len & 0xFF);
    out[3] = static_cast<uint8_t>(len >> 8);
    for (int i = 0; i < 4; ++i) {
        out[4 + i] = static_cast<uint8_t>((seq >> (8 * i)) & 0xFF);
    }
    if (len) {
        std::memcpy(out + kHeaderLen, payload, len);
    }
    const uint16_t crc = crc16(out + 1, kHeaderLen - 1 + len);
    out[kHeaderLen + len] = static_cast<uint8_t>(crc & 0xFF);
    out[kHeaderLen + len + 1] = static_cast<uint8_t>(crc >> 8);
    return total;
}

Writer& Writer::u8(uint8_t v) {
    if (n_ + 1 > cap_) {
        ok_ = false;
        return *this;
    }
    buf_[n_++] = v;
    return *this;
}

Writer& Writer::u16(uint16_t v) {
    return u8(static_cast<uint8_t>(v & 0xFF)).u8(static_cast<uint8_t>(v >> 8));
}

Writer& Writer::u32(uint32_t v) {
    return u16(static_cast<uint16_t>(v & 0xFFFF)).u16(static_cast<uint16_t>(v >> 16));
}

Writer& Writer::i64(int64_t v) {
    const uint64_t u = static_cast<uint64_t>(v);
    return u32(static_cast<uint32_t>(u & 0xFFFFFFFFu)).u32(static_cast<uint32_t>(u >> 32));
}

Writer& Writer::f32(float v) {
    uint32_t u;
    std::memcpy(&u, &v, sizeof(u));
    return u32(u);
}

Writer& Writer::f64(double v) {
    uint64_t u;
    std::memcpy(&u, &v, sizeof(u));
    return u32(static_cast<uint32_t>(u & 0xFFFFFFFFu)).u32(static_cast<uint32_t>(u >> 32));
}

Writer& Writer::bytes(const uint8_t* p, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) u8(p[i]);
    return *this;
}

Writer& Writer::zeros(std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) u8(0);
    return *this;
}

}  // namespace logfmt
}  // namespace gins
