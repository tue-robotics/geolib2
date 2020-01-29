#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/CompositeShape.h>
#include <geolib/datatypes.h>


TEST(TestSuite, testCase1){
    geo::CompositeShape comp;
    geo::Vector3 min1(-0.5, -0.5, -0.5);
    geo::Vector3 max1(0.5, 0.5, 0.5);
    geo::Box box1(min1, max1);

    geo::Pose3D p1(0.5, 0, 0);
    geo::Pose3D p2(-0.5, 0, 0);
    comp.addShape(box1, p1);
    comp.addShape(box1, p2);

    geo::Vector3 p(0.0, 0.0, 0.0);
    double radius = 0.5;

    ASSERT_TRUE(comp.intersect(p, radius));
}

int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
