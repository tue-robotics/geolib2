#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/datatypes.h>

#include "box_test.h"

TEST_F(BoxTest, Contains)
{
    ASSERT_TRUE(box.contains(origin));

    ASSERT_TRUE(box.contains(min));
    ASSERT_TRUE(box.contains(max));

    ASSERT_TRUE(box.contains(side_center));
    ASSERT_TRUE(box.contains(side_center_triangle));

    ASSERT_FALSE(box.contains(side_center_distance));

    ASSERT_FALSE(box.contains(side_center_close));
}

TEST_F(BoxTest, Intersect)
{
    ASSERT_TRUE(box.intersect(origin, 0));
    ASSERT_TRUE(box.intersect(origin, 0.1));

    ASSERT_TRUE(box.intersect(min, 0));
    ASSERT_TRUE(box.intersect(min, 0.1));
    ASSERT_TRUE(box.intersect(max, 0));
    ASSERT_TRUE(box.intersect(max, 0.1));

    ASSERT_TRUE(box.intersect(side_center, 0));
    ASSERT_TRUE(box.intersect(side_center, 0.1));
    ASSERT_TRUE(box.intersect(side_center_triangle, 0));
    ASSERT_TRUE(box.intersect(side_center_triangle, 0.1));

    ASSERT_FALSE(box.intersect(side_center_distance, 0));
    ASSERT_FALSE(box.intersect(side_center_distance, 0.1));
    ASSERT_TRUE(box.intersect(side_center_distance, 0.5));
    ASSERT_TRUE(box.intersect(side_center_distance, 0.6));

    ASSERT_FALSE(box.intersect(side_center_close, 0));
    ASSERT_FALSE(box.intersect(side_center_close, 0.1));
    ASSERT_TRUE(box.intersect(side_center_close, 0.2));
    ASSERT_TRUE(box.intersect(side_center_close, 0.25));
}

int main(int argc, char **argv) {
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
