#include "geolib/CompositeShape.h"

namespace geo {

CompositeShape::CompositeShape() {
}

CompositeShape::~CompositeShape() {
}

CompositeShape* CompositeShape::clone() const {
    return new CompositeShape(*this);
}

bool CompositeShape::intersect(const Ray &, float t0, float t1, double& distance) const {
    std::cout << "CompositeShape::intersect NOT YET IMPLEMENTED!" << std::endl;
    return false;
}

void CompositeShape::addShape(const Shape& shape, const Pose3D& pose) {
    const std::vector<Triangle>& triangles = shape.getMesh();

    for(std::vector<Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
       mesh_.push_back(geo::Triangle(pose * it->p1_, pose * it->p2_, pose * it->p3_));
    }
}

}
