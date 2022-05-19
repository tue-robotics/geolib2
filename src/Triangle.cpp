#include "geolib/Triangle.h"

namespace geo {

Triangle::Triangle(const Vector3& p1_, const Vector3& p2_, const Vector3& p3_) : m({p1_, p2_, p3_}) {
}

Triangle::~Triangle() {
}

std::ostream& operator<< (std::ostream& out, const Triangle& t) {
    out << "Triangle: [" << std::endl << "p1: " << t[0] << std::endl << "p2: " << t[1] << std::endl << "p3: " << t[2] << "]";
    return out;
}

}
