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
    geo::Shape* shapeptr;

    geo::CompositeShape comp;
    geo::Vector3 min1(-0.5, -0.5, -0.5);
    geo::Vector3 max1(0.5, 0.5, 0.5);
    geo::Box box1(min1, max1);

    geo::Pose3D p1(0.1, 0.1, 0.1); // easy offset pose
    geo::Pose3D p2(0.0, 0.0, 0.0); // exact overlap
    geo::Pose3D p3(1.0, 0.0, 0.0); // meet at one face == no intersect
    geo::Pose3D p2(2.0, 0.0, 0.0); // far away == no intersect
    geo::Pose3D p4(1.1, 0.0, 0.0, 0.0, 0.0, 1.05); // rotated pose which intersects

    shapeptr = &box1;

    ASSERT_TRUE(shapeptr->intersect(p1, box1));
    ASSERT_TRUE(shapeptr->intersect(p2, box1));
    ASSERT_FALSE(shapeptr->intersect(p3, box1));
    ASSERT_FALSE(shapeptr->intersect(p4, box1));
    ASSERT_TRUE(shapeptr->intersect(p5, box1));
}

int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
