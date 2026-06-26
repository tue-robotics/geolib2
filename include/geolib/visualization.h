#ifndef GEOLIB_SERIALIZATION_H_
#define GEOLIB_SERIALIZATION_H_

#include "datatypes.h"

#include <iostream>

namespace geo
{

class visualization
{

public:
    static void showKinect(const ShapeConstPtr& shape, double canvas_width, double canvas_height);
};

} // namespace geo

#endif
