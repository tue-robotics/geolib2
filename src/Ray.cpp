#include "geolib/Ray.h"

namespace geo {

Ray::Ray(const Vector3 &o, const Vector3 &d) : origin_(o), direction_(d.normalized()), length_(0)
{
    calculateInvDirection();

    sign_[0] = (direction_.x < 0);
    sign_[1] = (direction_.y < 0);
    sign_[2] = (direction_.z < 0);
}

void Ray::calculateInvDirection()
{
    // Hardcoded EPS
    double inv_x = std::abs(direction_.x) < 1e-9 ? 0 : 1/direction_.x;
    double inv_y = std::abs(direction_.y) < 1e-9 ? 0 : 1/direction_.y;
    double inv_z = std::abs(direction_.z) < 1e-9 ? 0 : 1/direction_.z;

    inv_direction_ = geo::Vector3(inv_x, inv_y, inv_z);
}

}
