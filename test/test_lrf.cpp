#include <gtest/gtest.h>

#include <geolib/sensors/LaserRangeFinder.h>
#include <geolib/datatypes.h>

#include <cmath>

double ANGLE_MIN = -M_PI_2;
double ANGLE_MAX = M_PI_2;
double RANGE_MIN = 0.2;
double RANGE_MAX = 60.0;
uint N_BEAMS = 9;

TEST(TestLRF, renderLineFront)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-M_PI, M_PI);
    lrf.setNumBeams(9);
    lrf.setRangeLimits(RANGE_MIN, RANGE_MAX);

    geo::Vec2d p1(1.0, -1.0);
    geo::Vec2d p2(1.0, 1.0);
    std::vector<double> ranges(N_BEAMS, RANGE_MAX);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(lrf.getAngleUpperIndex(0.0) -1, 4);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(0.0) -1], 1);
}

TEST(TestLRF, renderLineLeft)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-M_PI, M_PI);
    lrf.setNumBeams(9);
    lrf.setRangeLimits(RANGE_MIN, RANGE_MAX);

    geo::Vec2d p1(1.0, 1.0);
    geo::Vec2d p2(-1.0, 1.0);
    std::vector<double> ranges(N_BEAMS, RANGE_MAX);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(M_PI_2) -1], 1.0);
}

TEST(TestLRF, renderLineRight)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-M_PI, M_PI);
    lrf.setNumBeams(9);
    lrf.setRangeLimits(RANGE_MIN, RANGE_MAX);

    geo::Vec2d p1(-1.0, -1.0);
    geo::Vec2d p2(1.0, -1.0);
    std::vector<double> ranges(N_BEAMS, RANGE_MAX);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(-M_PI_2) -1], 1.0);
}

TEST(TestLRF, renderLineBack)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-M_PI, M_PI);
    lrf.setNumBeams(9);
    lrf.setRangeLimits(RANGE_MIN, RANGE_MAX);

    geo::Vec2d p1(-1.0, 1.0);
    geo::Vec2d p2(-1.0, -1.0);
    std::vector<double> ranges(N_BEAMS, RANGE_MAX);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(-M_PI) -1], 1.0);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(M_PI) - 1], 1.0);
}

TEST(TestLRF, renderLineWeird)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-M_PI_2, M_PI_2); // angle limits +- 90 degrees
    lrf.setNumBeams(30);
    lrf.setRangeLimits(RANGE_MIN, RANGE_MAX);

    geo::Vec2d p1(1.0, 1.5); // point in view of the robot
    geo::Vec2d p2(-2.0, -0.75); // in the blind spot of the robot at a negative angle ~ -2.78 RAD
    // the line connecting these two points passes behind the robot. But the difference in angle in view of the robot is less than 180%
    std::vector<double> ranges(N_BEAMS, RANGE_MAX);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(ranges[0], RANGE_MAX);
    ASSERT_LT(ranges[N_BEAMS-1], RANGE_MAX);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(p1.x, p1.y)-1], RANGE_MAX);
    ASSERT_LT(ranges[lrf.getAngleUpperIndex(p1.x, p1.y)], RANGE_MAX);

}

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

TEST(TestLRF, getAngleUpperIndexUnitCircle)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-M_PI, M_PI);
    lrf.setNumBeams(9);
    ASSERT_EQ(lrf.getAngleUpperIndex(-1.0, 0.0)-1, 8);
    ASSERT_EQ(lrf.getAngleUpperIndex(-1.0, -1.0)-1, 1);
    ASSERT_EQ(lrf.getAngleUpperIndex(0.0, -1.0)-1, 2);
    ASSERT_EQ(lrf.getAngleUpperIndex(1.0, -1.0)-1, 3);
    ASSERT_EQ(lrf.getAngleUpperIndex(1.0, 0.0)-1, 4);
    ASSERT_EQ(lrf.getAngleUpperIndex(1.0, 1.0)-1, 5);
    ASSERT_EQ(lrf.getAngleUpperIndex(0.0, 1.0)-1, 6);
    ASSERT_EQ(lrf.getAngleUpperIndex(-1.0, 1.0)-1, 7);
    ASSERT_EQ(lrf.getAngleUpperIndex(0.0, 0.0), 5);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
