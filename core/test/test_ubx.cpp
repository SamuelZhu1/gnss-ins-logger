#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "gins/ubx.hpp"

using namespace gins::ubx;

namespace {
void put32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}

std::vector<uint8_t> nav_pvt_frame() {
    uint8_t pl[92] = {};
    put32(pl + 0, 345'600'100);          // iTOW
    pl[20] = 3;                          // 3D fix
    pl[21] = 0x01 | (2 << 6);            // gnssFixOK, RTK fixed
    pl[23] = 27;                         // satellites
    put32(pl + 24, static_cast<uint32_t>(-1232460000));  // lon -123.246
    put32(pl + 28, 492606000);           // lat 49.2606
    put32(pl + 32, 80123);               // height mm
    put32(pl + 40, 14);                  // hAcc mm
    put32(pl + 44, 21);
    put32(pl + 48, 5200);                // velN mm/s
    put32(pl + 52, static_cast<uint32_t>(-1300));
    put32(pl + 56, 40);
    put32(pl + 68, 35);                  // sAcc
    std::vector<uint8_t> out(200);
    out.resize(build_frame(kClsNav, kIdNavPvt, pl, sizeof(pl), out.data(), out.size()));
    return out;
}
}  // namespace

TEST(Ubx, ParsesNavPvtAfterGarbageAndDecodesFields) {
    Parser p;
    const uint8_t junk[] = {0x00, 0xB5, 0x13, 0x24, 0xB5};  // includes a false sync
    for (uint8_t b : junk) EXPECT_FALSE(p.feed(b));
    const auto frame = nav_pvt_frame();
    bool done = false;
    for (uint8_t b : frame) done = p.feed(b);
    ASSERT_TRUE(done);
    EXPECT_EQ(p.cls(), kClsNav);
    EXPECT_EQ(p.id(), kIdNavPvt);

    NavPvt pvt;
    ASSERT_TRUE(decode_nav_pvt(p.payload(), p.len(), pvt));
    EXPECT_EQ(pvt.itow_ms, 345'600'100u);
    EXPECT_EQ(pvt.carr_soln, 2);
    const gins::GnssFix f = to_fix(pvt, 1234);
    EXPECT_NEAR(f.lat_deg, 49.2606, 1e-9);
    EXPECT_NEAR(f.lon_deg, -123.246, 1e-9);
    EXPECT_NEAR(f.h_m, 80.123, 1e-9);
    EXPECT_NEAR(f.vel_ned.y(), -1.3, 1e-6);
    EXPECT_NEAR(f.h_acc, 0.014, 1e-6);
    EXPECT_EQ(f.fix_type, 3);
}

TEST(Ubx, BadChecksumIsCountedAndNextFrameStillParses) {
    Parser p;
    auto bad = nav_pvt_frame();
    bad[30] ^= 0x40;
    for (uint8_t b : bad) EXPECT_FALSE(p.feed(b));
    EXPECT_EQ(p.checksum_errors(), 1u);
    bool done = false;
    for (uint8_t b : nav_pvt_frame()) done = p.feed(b);
    EXPECT_TRUE(done);
}

TEST(Ubx, FixWithoutGnssFixOkIsMarkedInvalid) {
    NavPvt pvt;
    pvt.fix_type = 3;
    pvt.flags = 0;
    EXPECT_EQ(to_fix(pvt, 0).fix_type, 0);
}

TEST(Ubx, ValsetUsesValueSizeFromKey) {
    const ConfigItem items[] = {{key::kRateMeas, 100}, {key::kUart1Baud, 460800}, {key::kUart1OutNmea, 0}};
    uint8_t out[64];
    const std::size_t n = build_valset(items, 3, out, sizeof(out));
    // 4-byte header + (4+2) + (4+4) + (4+1) payload, plus 8 bytes framing.
    EXPECT_EQ(n, 8u + 4 + 6 + 8 + 5);
    EXPECT_EQ(out[2], kClsCfg);
    EXPECT_EQ(out[3], kIdCfgValset);
    EXPECT_EQ(out[7], 0x01);  // RAM layer
    EXPECT_EQ(out[14], 100);  // rate value low byte after the 4-byte key
    EXPECT_EQ(out[15], 0);
    Parser p;
    bool done = false;
    for (std::size_t i = 0; i < n; ++i) done = p.feed(out[i]);
    EXPECT_TRUE(done);  // the checksum is right
}
