#include "wire_volume/Shape.h"

namespace vwm {

Shape::Shape() {

}

Shape::~Shape() {

}

const std::vector<Triangle>& Shape::getMesh() const {
    return mesh_;
}

}
