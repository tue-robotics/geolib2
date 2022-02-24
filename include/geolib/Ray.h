#ifndef GEOLIB_RAY_H_
#define GEOLIB_RAY_H_

#include "datatypes.h"

#include <array>

namespace geo {

class Ray {

public:

    Ray(const Vector3 &o, const Vector3 &d) ;

    const Vector3& getOrigin() const { return origin_; }

    const Vector3& getDirection() const { return direction_; }

    const Vector3& getInvDirection() const { return inv_direction_; }

    double getLength() const { return length_; }

    const std::array<int, 3>& getSign() const { return sign_; }

    void setLength(const double length) { length_ = length; }

protected:

    Vector3 origin_;

    Vector3 direction_;

    Vector3 inv_direction_;

    double length_; // When the length is zero, the ray is infinite

    std::array<int, 3> sign_;

    void calculateInvDirection();

};

}

#endif
