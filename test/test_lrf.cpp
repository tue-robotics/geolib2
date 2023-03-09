#include <gtest/gtest.h>

#include <geolib/sensors/LaserRangeFinder.h>
#include <geolib/datatypes.h>

#include <cmath>


class TestLRF : public testing::Test
{
protected:
    TestLRF(double angle_min_=-M_PI, double angle_max_=M_PI, double range_min_=0.2, double range_max_=60., uint n_beams_=9) :
        angle_min(angle_min_), angle_max(angle_max_), range_min(range_min_), range_max(range_max_), n_beams(n_beams_)
    {
    }

    virtual ~TestLRF()
    {
    }

    void SetUp() override
    {
        lrf.setAngleLimits(angle_min, angle_max);
        lrf.setNumBeams(n_beams);
        lrf.setRangeLimits(range_min, range_max);

        ranges.resize(n_beams, range_max);
    }

    geo::LaserRangeFinder lrf;

    const double angle_min;
    const double angle_max;
    const double range_min;
    const double range_max;
    const uint n_beams;

    std::vector<double> ranges;
};

class TestLRF2 : public TestLRF
{
protected:
    TestLRF2() : TestLRF(-M_PI_2, M_PI_2)
    {
    }

    virtual ~TestLRF2()
    {
    }
};

class TestLRF3 : public TestLRF
{
protected:
    TestLRF3() : TestLRF(-M_PI_2, M_PI_2, 0.2, 60., 30)
    {
    }

    virtual ~TestLRF3()
    {
    }
};

TEST_F(TestLRF, renderLineFront)
{
    geo::Vec2d p1(1.0, -1.0);
    geo::Vec2d p2(1.0, 1.0);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(lrf.getAngleUpperIndex(0.0) -1, 4);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(0.0) -1], 1);
}

TEST_F(TestLRF, renderLineLeft)
{
    geo::Vec2d p1(1.0, 1.0);
    geo::Vec2d p2(-1.0, 1.0);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(M_PI_2) -1], 1.0);
}

TEST_F(TestLRF, renderLineRight)
{
    geo::Vec2d p1(-1.0, -1.0);
    geo::Vec2d p2(1.0, -1.0);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(-M_PI_2) -1], 1.0);
}

TEST_F(TestLRF, renderLineBack)
{
    geo::Vec2d p1(-1.0, 1.0);
    geo::Vec2d p2(-1.0, -1.0);

    lrf.renderLine(p1, p2, ranges);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(-M_PI) -1], 1.0);
    ASSERT_EQ(ranges[lrf.getAngleUpperIndex(M_PI) - 1], 1.0);
}

TEST_F(TestLRF3, renderLineWeird)
{
    double a1 = angle_max - 0.1; // Point in view of the robot -> angle_min < a1 < angle_max
    double a2 = angle_min - 1.0; // Point in the blind spot of the robot but with a positive angle -> a2 > 0

    // The line connecting these two points should pass behind the robot -> a1 - a2 > M_PI
    // However the difference between the a1 and the lower angle limit is less than half a circle -> a1 + a_max < M_PI
    geo::Vec2d p1(cos(a1), sin(a1));
    geo::Vec2d p2(cos(a2), sin(a2));

    lrf.renderLine(p1, p2, ranges);

    uint upper_index_a1 = lrf.getAngleUpperIndex(a1);
    // The rendered line passes behind the robot. But not through the blindspot. Therefore the first index should remain untouched
    for (uint i = 0; i<upper_index_a1-1; ++i)
    {
        ASSERT_EQ(ranges[i], range_max) << "Range at index [" << i << "] should not be rendered. Instead is rendered to " << ranges[i];
    }
    // At a1 we should see the switch between rendered and not rendered. indices above a1 should be rendered. Because points lie on a unit circle they should be <= 1
    for (uint i = upper_index_a1; i<n_beams; ++i)
    {
        ASSERT_LE(ranges[i], 1.) << "Range at index [" << i << "] should be rendered to <=1. Instead its value is " << ranges[i];
    }
}

TEST_F(TestLRF3, renderLineRight2)
{
    geo::Vec2d p1(-1.0, -1.0);
    geo::Vec2d p2(1.0, -1.0);

    lrf.renderLine(p1, p2, ranges);

    uint upper_index_p2 = lrf.getAngleUpperIndex(p2.x, p2.y);
    // The rendered line passes the robot on the right and ends in the blindspot.
    for (uint i = 0; i<upper_index_p2-1; ++i)
    {
        ASSERT_LE(ranges[i], sqrt(2)) << "Range at index [" << i << "] should be rendered to <=" << sqrt(2) << ". Instead its value is " << ranges[i];
    }
    for (uint i = upper_index_p2; i<n_beams; ++i)
    {
        ASSERT_EQ(ranges[i], range_max) << "Range at index [" << i << "] should not be rendered. Instead is rendered to " << ranges[i];
    }
}

TEST_F(TestLRF2, getAngleUpperIndexAngle)
{
    ASSERT_EQ(lrf.getAngleUpperIndex(1.5*angle_min), 0);
    ASSERT_EQ(lrf.getAngleUpperIndex(angle_min), 1);
    ASSERT_EQ(lrf.getAngleUpperIndex(0.5*angle_min + 0.5*angle_max), std::floor(0.5*n_beams-0.5)+1);
    ASSERT_EQ(lrf.getAngleUpperIndex(angle_max), n_beams);
    ASSERT_EQ(lrf.getAngleUpperIndex(1.5*angle_max), n_beams);
}

TEST_F(TestLRF2, getAngleUpperIndexXY)
{
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(1.5*angle_min), sin(1.5*angle_min)), 0);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(angle_min), sin(angle_min)), 1);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(0.5*angle_min + 0.5*angle_max), sin(0.5*angle_min + 0.5*angle_max)), std::floor(0.5*n_beams - 0.5) + 1);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(angle_max), sin(angle_max)), n_beams);
    ASSERT_EQ(lrf.getAngleUpperIndex(cos(1.5*angle_max), sin(1.5*angle_max)), n_beams);
}

TEST_F(TestLRF, getAngleUpperIndexUnitCircle)
{
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
