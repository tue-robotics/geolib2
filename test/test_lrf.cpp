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
    lrf.setNumBeams(30);
    lrf.setRangeLimits(RANGE_MIN, RANGE_MAX);

    double a_max = M_PI_2;
    double a1 = a_max - 0.1; // point in view of the robot -> -a_max < a1 < a_max
    double a2 = -a_max - 1.0; // in the blind spot of the robot but with a positive angle -> a2 > 0
    // the line connecting these two points should pass behind the robot -> a1 + a2 > M_PI

    lrf.setAngleLimits(-a_max, a_max);

    geo::Vec2d p1(cos(a1), sin(a1)); 
    geo::Vec2d p2(cos(a2), sin(a2)); 
    std::vector<double> ranges(N_BEAMS, RANGE_MAX);

    lrf.renderLine(p1, p2, ranges);

    // the rendered line passes behind the robot. But not through the blindspot. Therefore the first index should remain untouched
    ASSERT_EQ(ranges[0], RANGE_MAX);
    // the rendered line passes behind the robot. So the last index should be rendered. i.e. less than 1.0
    ASSERT_LT(ranges[N_BEAMS-1], 1.0);
    // At a2 we should see the switch between rendered and not rendered. indices above a2 should be rendered.
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(a1)-1], RANGE_MAX);
    ASSERT_LT(ranges[lrf.getAngleUpperIndex(a1)], 1.0);
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
