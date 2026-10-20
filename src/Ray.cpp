#include "geolib/Ray.h"
#include "geolib/datatypes.h"
#include <cmath>
#include <limits>

namespace geo
{

Ray::Ray(const Vector3& o, const Vector3& d) : origin_(o), direction_(d.normalized()), length_(0)
{
    calculateInvDirection();

    sign_[0] = (inv_direction_.x < 0);
    sign_[1] = (inv_direction_.y < 0);
    sign_[2] = (inv_direction_.z < 0);
}

void Ray::calculateInvDirection()
{
    // A zero component results in a signed infinite inverse, which slab tests rely on to detect rays that are parallel
    // to an axis. Small, but non-zero components are not parallel, so they get their exact (large) inverse.
    double const inf = std::numeric_limits<double>::infinity();
    double const inv_x = direction_.x == 0 ? std::copysign(inf, direction_.x) : 1 / direction_.x;
    double const inv_y = direction_.y == 0 ? std::copysign(inf, direction_.y) : 1 / direction_.y;
    double const inv_z = direction_.z == 0 ? std::copysign(inf, direction_.z) : 1 / direction_.z;

    inv_direction_ = geo::Vector3(inv_x, inv_y, inv_z);
}

} // namespace geo
