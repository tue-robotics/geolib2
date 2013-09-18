#ifndef GEOLIB_TRIANGLE_H_
#define GEOLIB_TRIANGLE_H_

#include "datatypes.h"

namespace geo {

class Triangle {

public:

    Triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3);

    virtual ~Triangle();

    Vector3 p1_, p2_, p3_;

};

}

#endif
