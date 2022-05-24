#include <gtest/gtest.h>

#include <geolib/Shape.h>

#include "box_test.h"


class ShapeTest : public BoxTest
{
protected:
    ShapeTest() : shape(box)
    {
    }

    virtual ~ShapeTest()
    {
    }

    geo::Shape shape;
};

TEST_F(ShapeTest, Contains)
{
    ASSERT_TRUE(shape.contains(origin));

    ASSERT_TRUE(shape.contains(min));
    ASSERT_TRUE(shape.contains(max));

    ASSERT_TRUE(shape.contains(side_center));

    ASSERT_FALSE(shape.contains(side_center_distance));

    ASSERT_FALSE(shape.contains(side_center_close));
}

TEST_F(ShapeTest, Intersect)
{
    ASSERT_TRUE(shape.intersect(origin, 0));
    ASSERT_TRUE(shape.intersect(origin, 0.1));

    ASSERT_TRUE(shape.intersect(min, 0));
    ASSERT_TRUE(shape.intersect(min, 0.1));
    ASSERT_TRUE(shape.intersect(max, 0));
    ASSERT_TRUE(shape.intersect(max, 0.1));

    ASSERT_TRUE(shape.intersect(side_center, 0));
    ASSERT_TRUE(shape.intersect(side_center, 0.1));

    ASSERT_FALSE(shape.intersect(side_center_distance, 0));
    ASSERT_FALSE(shape.intersect(side_center_distance, 0.1));
    ASSERT_TRUE(shape.intersect(side_center_distance, 0.5));
    ASSERT_TRUE(shape.intersect(side_center_distance, 0.6));

    ASSERT_FALSE(shape.intersect(side_center_close, 0));
    ASSERT_FALSE(shape.intersect(side_center_close, 0.1));
    ASSERT_TRUE(shape.intersect(side_center_close, 0.2));
    ASSERT_TRUE(shape.intersect(side_center_close, 0.25));
}

int main(int argc, char **argv) {
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
