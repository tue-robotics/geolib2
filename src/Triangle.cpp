#include "geolib/Triangle.h"

namespace geo {

Triangle::Triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3)
    : p1_(p1), p2_(p2), p3_(p3) {
}

Triangle::~Triangle() {

}

}
