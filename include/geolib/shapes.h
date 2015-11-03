#ifndef GEOLIB_SHAPES_H_
#define GEOLIB_SHAPES_H_

#include "geolib/datatypes.h"

namespace geo
{

void createCylinder(geo::Shape& shape, double radius, double height, int num_corners = 20);

// The points should be in clock-wise order, e.g., [ (-1, 1), (1, 1), (1, -1), (-1, -1) ]
void createConvexPolygon(geo::Shape& shape, const std::vector<geo::Vec2>& points, double height);

} // end namespace geo

#endif
