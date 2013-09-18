#ifndef GEOLIB_RAY_H_
#define GEOLIB_RAY_H_

#include "datatypes.h"

namespace geo {

class Ray {

public:

    Ray(const Vector3 &o, const Vector3 &d) ;

    Vector3 origin_;

    Vector3 direction_;

    Vector3 inv_direction_;

    double length_;

    int sign[3];

};

}

#endif
