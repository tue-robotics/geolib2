#include <gtest/gtest.h>

#include <geolib/sensors/LaserRangeFinder.h>

#include <cmath>

double ANGLE_MIN = -M_PI_2;
double ANGLE_MAX = M_PI_2;
uint N_BEAMS = 10;

TEST(TestLRF, getAngleUpperIndexAngle)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(ANGLE_MIN, ANGLE_MAX);
    lrf.setNumBeams(N_BEAMS);
    ASSERT_EQ(lrf.getAngleUpperIndex(1.5*ANGLE_MIN), 0);
    ASSERT_EQ(lrf.getAngleUpperIndex(1.5*ANGLE_MAX), N_BEAMS);
}

TEST(TestLRF, getAngleUpperIndexXY)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(ANGLE_MIN, ANGLE_MAX);
    lrf.setNumBeams(N_BEAMS);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(1.5*ANGLE_MIN), sin(1.5*ANGLE_MIN)), 0);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(1.5*ANGLE_MAX), sin(1.5*ANGLE_MAX)), N_BEAMS);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
