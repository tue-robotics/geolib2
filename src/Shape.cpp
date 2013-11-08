#include "geolib/Shape.h"

namespace geo {

Shape::Shape() {

}

Shape::~Shape() {

}

const std::vector<Triangle>& Shape::getMesh() const {
    return mesh_;
}

double Shape::getMaxRadius() const {
    return 0;
}

}
