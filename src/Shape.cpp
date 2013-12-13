#include "geolib/Shape.h"

namespace geo {

Shape::Shape() {

}

Shape::~Shape() {

}

Shape* Shape::clone() const {
    return new Shape(*this);
}

bool Shape::intersect(const Ray &, float t0, float t1, double& distance) const {
    return false;
}

const Mesh& Shape::getMesh() const {
    return mesh_;
}

double Shape::getMaxRadius() const {
    return 0;
}

}
