#include <gtest/gtest.h>

#include <geolib/sensors/LaserRangeFinder.h>

#include <cmath>

double ANGLE_MIN = -M_PI_2;
double ANGLE_MAX = M_PI_2;
uint N_BEAMS = 9;

TEST(TestLRF, getAngleUpperIndexAngle)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(ANGLE_MIN, ANGLE_MAX);
    lrf.setNumBeams(N_BEAMS);
    ASSERT_EQ(lrf.getAngleUpperIndex(1.5*ANGLE_MIN), 0);
    ASSERT_EQ(lrf.getAngleUpperIndex(ANGLE_MIN), 1);
    ASSERT_EQ(lrf.getAngleUpperIndex(0.5*ANGLE_MIN + 0.5*ANGLE_MAX), std::floor(0.5*N_BEAMS-0.5)+1);
    ASSERT_EQ(lrf.getAngleUpperIndex(ANGLE_MAX), N_BEAMS);
    ASSERT_EQ(lrf.getAngleUpperIndex(1.5*ANGLE_MAX), N_BEAMS);
}

TEST(TestLRF, getAngleUpperIndexXY)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(ANGLE_MIN, ANGLE_MAX);
    lrf.setNumBeams(N_BEAMS);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(1.5*ANGLE_MIN), sin(1.5*ANGLE_MIN)), 0);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(ANGLE_MIN), sin(ANGLE_MIN)), 1);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(0.5*ANGLE_MIN + 0.5*ANGLE_MAX), sin(0.5*ANGLE_MIN + 0.5*ANGLE_MAX)), std::floor(0.5*N_BEAMS-0.5)+1);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(ANGLE_MAX), sin(ANGLE_MAX)), N_BEAMS);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(1.5*ANGLE_MAX), sin(1.5*ANGLE_MAX)), N_BEAMS);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
