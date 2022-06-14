#include "geolib/Triangle.h"

namespace geo {

double triangleArea(const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
   return 0.5 * ((p1-p2).cross(p1-p3)).length();
}

Triangle::Triangle(const Vector3& p1_, const Vector3& p2_, const Vector3& p3_) : m({p1_, p2_, p3_}) {
}

Triangle::~Triangle() {
}

double Triangle::area() const {
    return triangleArea(p1(), p2(), p3());
}

std::ostream& operator<< (std::ostream& out, const Triangle& t) {
    out << "Triangle: [" << std::endl << "p1: " << t[0] << std::endl << "p2: " << t[1] << std::endl << "p3: " << t[2] << "]";
    return out;
}

}
