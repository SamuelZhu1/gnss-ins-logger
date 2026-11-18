#include <gtest/gtest.h>

#include <vector>

#include "gins/log_format.hpp"

using namespace gins::logfmt;

namespace {
std::vector<uint8_t> record(Type t, uint32_t seq, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> out(payload.size() + kOverhead);
    out.resize(encode(t, seq, payload.data(), static_cast<uint16_t>(payload.size()), out.data(), out.size()));
    return out;
}
struct Got {
    Type type;
    uint32_t seq;
    std::vector<uint8_t> payload;
};
std::vector<Got> read_all(const std::vector<uint8_t>& bytes, std::size_t chunk, Reader& r) {
    std::vector<Got> got;
    for (std::size_t i = 0; i < bytes.size(); i += chunk) {
        const std::size_t n = std::min(chunk, bytes.size() - i);
        r.feed(bytes.data() + i, n, [&](const Reader::Record& rec) {
            got.push_back({rec.type, rec.seq, std::vector<uint8_t>(rec.payload, rec.payload + rec.len)});
        });
    }
    r.finish([&](const Reader::Record& rec) {
        got.push_back({rec.type, rec.seq, std::vector<uint8_t>(rec.payload, rec.payload + rec.len)});
    });
    return got;
}
}  // namespace

TEST(LogFormat, Crc16MatchesCcittFalseCheckValue) {
    const uint8_t msg[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(crc16(msg, sizeof(msg)), 0x29B1);
}

TEST(LogFormat, WriterIsLittleEndian) {
    uint8_t buf[32];
    Writer w(buf, sizeof(buf));
    w.u16(0x1234).i32(-2).i64(0x0102030405060708LL).f32(1.0f);
    ASSERT_TRUE(w.ok());
    EXPECT_EQ(w.size(), 2u + 4 + 8 + 4);
    EXPECT_EQ(buf[0], 0x34);
    EXPECT_EQ(buf[2], 0xFE);
    EXPECT_EQ(buf[6], 0x08);
    EXPECT_EQ(buf[17], 0x3F);  // 1.0f = 0x3F800000
    Writer small(buf, 3);
    small.u32(1);
    EXPECT_FALSE(small.ok());
}

TEST(LogFormat, RoundTripsInAnyChunkSize) {
    std::vector<uint8_t> bytes;
    for (uint32_t i = 0; i < 50; ++i) {
        auto r = record(i % 2 ? Type::kImu : Type::kPps, i, std::vector<uint8_t>(i % 2 ? 22 : 8, static_cast<uint8_t>(i)));
        bytes.insert(bytes.end(), r.begin(), r.end());
    }
    for (std::size_t chunk : {1u, 7u, 512u, 100000u}) {
        Reader r;
        const auto got = read_all(bytes, chunk, r);
        ASSERT_EQ(got.size(), 50u) << "chunk " << chunk;
        EXPECT_EQ(got[13].seq, 13u);
        EXPECT_EQ(got[13].type, Type::kImu);
        EXPECT_EQ(got[13].payload.size(), 22u);
        EXPECT_EQ(r.crc_errors(), 0u);
    }
}

TEST(LogFormat, CorruptRecordCostsOnlyItself) {
    std::vector<uint8_t> bytes;
    for (uint32_t i = 0; i < 3; ++i) {
        auto r = record(Type::kImu, i, std::vector<uint8_t>(22, 0xA5));  // payload full of sync bytes
        bytes.insert(bytes.end(), r.begin(), r.end());
    }
    bytes[32 + 12] ^= 0x01;              // flip a bit inside the second record
    bytes.insert(bytes.begin(), {0x11, 0xA5, 0x22});  // junk before the first
    Reader r;
    const auto got = read_all(bytes, 5, r);
    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(got[0].seq, 0u);
    EXPECT_EQ(got[1].seq, 2u);
    EXPECT_GE(r.crc_errors(), 1u);
}
