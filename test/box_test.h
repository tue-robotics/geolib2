#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/datatypes.h>

class BoxTest : public testing::Test
{
protected:
    BoxTest() : min(-0.5, -0.5, -0.5), max(0.5, 0.5, 0.5), origin(0, 0, 0), box(min, max)
    {
        side_center = geo::Vector3(0, 0, 0.5);
        side_center_triangle = geo::Vector3(0, 0.25, 0.5);
        side_center_close = geo::Vector3(0, 0, 0.7); // Closer than max radius
        side_center_distance = geo::Vector3(0, 0, 1);
    }

    virtual ~BoxTest()
    {
    }

    geo::Vector3 min, max, origin;
    geo::Box box;

    geo::Vector3 side_center, side_center_triangle, side_center_close, side_center_distance;
};
