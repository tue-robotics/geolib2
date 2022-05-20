#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/CompositeShape.h>
#include <geolib/Shape.h>
#include <geolib/datatypes.h>


TEST(TestIntersect, Box)
{
    geo::Vector3 min(-0.5, -0.5, -0.5);
    geo::Vector3 max(0.5, 0.5, 0.5);
    geo::Box box(min, max);

    geo::Vector3 origin(0, 0, 0);
    ASSERT_TRUE(box.contains(origin));
    ASSERT_TRUE(box.intersect(origin, 0));
    ASSERT_TRUE(box.intersect(origin, 0.1));

    ASSERT_TRUE(box.contains(min));
    ASSERT_TRUE(box.intersect(min, 0));
    ASSERT_TRUE(box.intersect(min, 0.1));
    ASSERT_TRUE(box.intersect(max, 0));
    ASSERT_TRUE(box.contains(max));
    ASSERT_TRUE(box.intersect(max, 0.1));

    geo::Vector3 side_center(0, 0, 0.5);
    ASSERT_TRUE(box.contains(side_center));
    ASSERT_TRUE(box.intersect(side_center, 0));
    ASSERT_TRUE(box.intersect(side_center, 0.1));

    geo::Vector3 side_center_distance(0, 0, 1);
    ASSERT_FALSE(box.contains(side_center_distance));
    ASSERT_FALSE(box.intersect(side_center_distance, 0));
    ASSERT_FALSE(box.intersect(side_center_distance, 0.1));
    ASSERT_TRUE(box.intersect(side_center_distance, 0.5));
    ASSERT_TRUE(box.intersect(side_center_distance, 0.6));

    geo::Vector3 side_center_close(0, 0, 0.7);
    ASSERT_FALSE(box.contains(side_center_close));
    ASSERT_FALSE(box.intersect(side_center_close, 0));
    ASSERT_FALSE(box.intersect(side_center_close, 0.1));
    ASSERT_TRUE(box.intersect(side_center_close, 0.2));
    ASSERT_TRUE(box.intersect(side_center_close, 0.25));
}

TEST(TestIntersect, Box_Composite)
{
    geo::CompositeShape comp;
    geo::Vector3 min(-0.5, -0.5, -0.5);
    geo::Vector3 max(0.5, 0.5, 0.5);
    geo::Box box(min, max);

    comp.addShape(box, geo::Pose3D::identity());

    geo::Vector3 origin(0, 0, 0);
    ASSERT_TRUE(comp.contains(origin));
    ASSERT_TRUE(comp.intersect(origin, 0));
    ASSERT_TRUE(comp.intersect(origin, 0.1));

    ASSERT_TRUE(comp.contains(min));
    ASSERT_TRUE(comp.intersect(min, 0));
    ASSERT_TRUE(comp.intersect(min, 0.1));
    ASSERT_TRUE(comp.intersect(max, 0));
    ASSERT_TRUE(comp.contains(max));
    ASSERT_TRUE(comp.intersect(max, 0.1));

    geo::Vector3 side_center(0, 0, 0.5);
    ASSERT_TRUE(comp.contains(side_center));
    ASSERT_TRUE(comp.intersect(side_center, 0));
    ASSERT_TRUE(comp.intersect(side_center, 0.1));

    geo::Vector3 side_center_distance(0, 0, 1);
    ASSERT_FALSE(comp.contains(side_center_distance));
    ASSERT_FALSE(comp.intersect(side_center_distance, 0));
    ASSERT_FALSE(comp.intersect(side_center_distance, 0.1));
    ASSERT_TRUE(comp.intersect(side_center_distance, 0.5));
    ASSERT_TRUE(comp.intersect(side_center_distance, 0.6));

    geo::Vector3 side_center_close(0, 0, 0.7);
    ASSERT_FALSE(comp.contains(side_center_close));
    ASSERT_FALSE(comp.intersect(side_center_close, 0));
    ASSERT_FALSE(comp.intersect(side_center_close, 0.1));
    ASSERT_TRUE(comp.intersect(side_center_close, 0.2));
    ASSERT_TRUE(comp.intersect(side_center_close, 0.25));
}

TEST(TestIntersect, Shape)
{
    geo::Vector3 min(-0.5, -0.5, -0.5);
    geo::Vector3 max(0.5, 0.5, 0.5);
    geo::Box box(min, max);
    geo::Shape shape(box);

    geo::Vector3 origin(0, 0, 0);
    ASSERT_TRUE(shape.contains(origin));
    ASSERT_TRUE(shape.intersect(origin, 0));
    ASSERT_TRUE(shape.intersect(origin, 0.1));

    ASSERT_TRUE(shape.contains(min));
    ASSERT_TRUE(shape.intersect(min, 0));
    ASSERT_TRUE(shape.intersect(min, 0.1));
    ASSERT_TRUE(shape.intersect(max, 0));
    ASSERT_TRUE(shape.contains(max));
    ASSERT_TRUE(shape.intersect(max, 0.1));

    geo::Vector3 side_center(0, 0, 0.5);
    ASSERT_TRUE(shape.contains(side_center));
    ASSERT_TRUE(shape.intersect(side_center, 0));
    ASSERT_TRUE(shape.intersect(side_center, 0.1));

    geo::Vector3 side_center_distance(0, 0, 1);
    ASSERT_FALSE(shape.contains(side_center_distance));
    ASSERT_FALSE(shape.intersect(side_center_distance, 0));
    ASSERT_FALSE(shape.intersect(side_center_distance, 0.1));
    ASSERT_TRUE(shape.intersect(side_center_distance, 0.5));
    ASSERT_TRUE(shape.intersect(side_center_distance, 0.6));

    geo::Vector3 side_center_close(0, 0, 0.7);
    ASSERT_FALSE(shape.contains(side_center_close));
    ASSERT_FALSE(shape.intersect(side_center_close, 0));
    ASSERT_FALSE(shape.intersect(side_center_close, 0.1));
    ASSERT_TRUE(shape.intersect(side_center_close, 0.2));
    ASSERT_TRUE(shape.intersect(side_center_close, 0.25));
}

int main(int argc, char **argv) {
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
