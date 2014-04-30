#include "geolib/Ray.h"

namespace geo {

Ray::Ray(const Vector3 &o, const Vector3 &d)
    : origin_(o),
      direction_(d), // ASSUME d is normalized!
      length_(0) {   // length_ == 0 means infinite ray

    inv_direction_ = geo::Vector3(1/direction_.x, 1/direction_.y, 1/direction_.z);
    sign[0] = (inv_direction_.x < 0);
    sign[1] = (inv_direction_.y < 0);
    sign[2] = (inv_direction_.z < 0);
}

}

