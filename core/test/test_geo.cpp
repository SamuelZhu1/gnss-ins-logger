#include <gtest/gtest.h>

#include "gins/geo.hpp"

using gins::Lla;

TEST(Geo, EcefRoundTripIsSubMillimetre) {
    for (const Lla& p : {Lla{49.2606, -123.2460, 80.0}, Lla{-33.9, 151.2, 12.0}, Lla{0.0, 0.0, 0.0},
                         Lla{89.9, 10.0, 3000.0}}) {
        const Lla q = gins::ecef_to_lla(gins::lla_to_ecef(p));
        EXPECT_NEAR(q.lat_deg, p.lat_deg, 1e-9);
        EXPECT_NEAR(q.lon_deg, p.lon_deg, 1e-9);
        EXPECT_NEAR(q.h_m, p.h_m, 1e-4);
    }
}

TEST(Geo, LocalFrameAxesPointNorthEastDown) {
    const gins::LocalFrame f({49.26, -123.25, 50.0});
    // About 111.2 km per degree of latitude here; 0.001 deg north is about 111 m.
    const auto n = f.to_ned({49.261, -123.25, 50.0});
    EXPECT_NEAR(n.x(), 111.2, 0.3);
    EXPECT_NEAR(n.y(), 0.0, 0.01);
    const auto e = f.to_ned({49.26, -123.249, 50.0});
    EXPECT_GT(e.y(), 70.0);  // 0.001 deg of longitude is shorter at 49 N (cos 49 = 0.65)
    EXPECT_NEAR(e.x(), 0.0, 0.01);
    const auto d = f.to_ned({49.26, -123.25, 40.0});
    EXPECT_NEAR(d.z(), 10.0, 1e-3);
}

TEST(Geo, LocalFrameRoundTrip) {
    const gins::LocalFrame f({49.26, -123.25, 50.0});
    const Eigen::Vector3d ned(-2500.0, 4100.0, -12.0);
    EXPECT_LT((f.to_ned(f.to_lla(ned)) - ned).norm(), 1e-6);
}

TEST(Geo, NormalGravityAtVancouver) {
    EXPECT_NEAR(gins::normal_gravity(49.26, 0.0), 9.8100, 0.0002);
    EXPECT_LT(gins::normal_gravity(49.26, 1000.0), gins::normal_gravity(49.26, 0.0));
}
