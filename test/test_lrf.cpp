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
    // the line connecting these two points should pass behind the robot -> a1 - a2 > M_PI
    // However the difference between the a1 and the lower angle limit is less than half a circle -> a1 + a_max < M_PI

    lrf.setAngleLimits(-a_max, a_max);

    geo::Vec2d p1(cos(a1), sin(a1)); 
    geo::Vec2d p2(cos(a2), sin(a2)); 
    std::vector<double> ranges(N_BEAMS, RANGE_MAX);

    lrf.renderLine(p1, p2, ranges);

    uint upper_index_a1 = lrf.getAngleUpperIndex(a1);
    // the rendered line passes behind the robot. But not through the blindspot. Therefore the first index should remain untouched
    for (uint i = 0; i<upper_index_a1-1; ++i)
    {
        ASSERT_EQ(ranges[i], RANGE_MAX) << "range at index [" << i << "] should not be rendered. Instead is rendered to " << ranges[i];
    }
    // At a1 we should see the switch between rendered and not rendered. indices above a1 should be rendered. Because points lie on a unit circle they should be <= 1
    for (uint i = upper_index_a1; i<N_BEAMS; ++i)
    {
        ASSERT_LE(ranges[i], 1.0) << "range at index [" << i << "] should be rendered to <=1. Instead its value is " << ranges[i];
    }
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
