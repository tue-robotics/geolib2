#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/CompositeShape.h>
#include <geolib/datatypes.h>

// Test intersect with radius
TEST(TestSuite, testCase1){
    geo::CompositeShape comp;
    geo::Vector3 min1(-0.5, -0.5, -0.5);
    geo::Vector3 max1(0.5, 0.5, 0.5);
    geo::Box box1(min1, max1);

    geo::Pose3D p1(0.5, 0, 0);
    geo::Pose3D p2(-0.5, 0, 0);
    comp.addShape(box1, p1);
    comp.addShape(box1, p2);

    geo::Vector3 v1(0.0, 0.0, 0.0);
    geo::Vector3 v2(10.0, 0.0, 0.0);
    double radius = 0.5;

    ASSERT_TRUE(comp.intersect(p, radius));
    ASSERT_FALSE(comp.intersect(p, radius));
}

// Test intersection with another shape
TEST(TestSuite, testCase2){
    geo::CompositeShape comp;
    geo::Vector3 min1(-0.5, -0.5, -0.5);
    geo::Vector3 max1(0.5, 0.5, 0.5);
    geo::Box box1(min1, max1);

    geo::Pose3D p1(0.5, 0, 0);
    geo::Pose3D p2(-0.5, 0, 0);
    comp.addShape(box1, p1);
    comp.addShape(box1, p2);

    geo::Pose3D p3(0.0, 0.0, 0.0);
    geo::Pose3D p4(10.0, 0.0, 0.0);

    ASSERT_TRUE(comp.intersect(p3, box1));
    ASSERT_FALSE(comp.intersect(p4, box1));
}

int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
