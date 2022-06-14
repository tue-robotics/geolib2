#include <gtest/gtest.h>

#include <geolib/CompositeShape.h>

#include "box_test.h"

class CompShapeTest : public BoxTest
{
protected:
    CompShapeTest()
    {
        comp.addShape(box, geo::Pose3D::identity());
    }

    virtual ~CompShapeTest()
    {
    }

    geo::CompositeShape comp;
};

TEST_F(CompShapeTest, Contains)
{
    ASSERT_TRUE(comp.contains(origin));

    ASSERT_TRUE(comp.contains(min));
    ASSERT_TRUE(comp.contains(max));

    ASSERT_TRUE(comp.contains(side_center));
    ASSERT_TRUE(comp.contains(side_center_triangle));

    ASSERT_FALSE(comp.contains(side_center_distance));

    ASSERT_FALSE(comp.contains(side_center_close));
}

TEST_F(CompShapeTest, Intersect)
{
    ASSERT_TRUE(comp.intersect(origin, 0));
    ASSERT_TRUE(comp.intersect(origin, 0.1));

    ASSERT_TRUE(comp.intersect(min, 0));
    ASSERT_TRUE(comp.intersect(min, 0.1));
    ASSERT_TRUE(comp.intersect(max, 0));
    ASSERT_TRUE(comp.intersect(max, 0.1));

    ASSERT_TRUE(comp.intersect(side_center, 0));
    ASSERT_TRUE(comp.intersect(side_center, 0.1));
    ASSERT_TRUE(comp.intersect(side_center_triangle, 0));
    ASSERT_TRUE(comp.intersect(side_center_triangle, 0.1));

    ASSERT_FALSE(comp.intersect(side_center_distance, 0));
    ASSERT_FALSE(comp.intersect(side_center_distance, 0.1));
    ASSERT_TRUE(comp.intersect(side_center_distance, 0.5));
    ASSERT_TRUE(comp.intersect(side_center_distance, 0.6));

    ASSERT_FALSE(comp.intersect(side_center_close, 0));
    ASSERT_FALSE(comp.intersect(side_center_close, 0.1));
    ASSERT_TRUE(comp.intersect(side_center_close, 0.2));
    ASSERT_TRUE(comp.intersect(side_center_close, 0.25));
}

int main(int argc, char **argv) {
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
