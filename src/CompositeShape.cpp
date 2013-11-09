#include "geolib/CompositeShape.h"

namespace geo {

CompositeShape::CompositeShape() {
}

CompositeShape::~CompositeShape() {
}

CompositeShape* CompositeShape::clone() const {
    return new CompositeShape(*this);
}

bool CompositeShape::intersect(const Ray& r, float t0, float t1, double& distance) const {

    bool hit = false;
    double min_distance = t1;
    for(std::vector<std::pair<ShapePtr, tf::Transform> >::const_iterator it = shapes_.begin(); it != shapes_.end(); ++it) {
        const Shape& shape = *it->first;
        const tf::Transform& pose_inv = it->second;

        Ray r_t(pose_inv * r.origin_, pose_inv.getBasis() * r.direction_);

        double d;
        if (shape.intersect(r_t, t0, min_distance, d)) {
            min_distance = d;
            hit = true;
        }
    }

    if (hit) {
        distance = min_distance;
        return true;
    }

    return false;
}

void CompositeShape::addShape(const Shape& shape, const Pose3D& pose) {
    // add to shapes
    shapes_.push_back(std::pair<ShapePtr, tf::Transform>(ShapePtr(shape.clone()), pose.inverse()));

    // add to mesh
    const std::vector<Triangle>& triangles = shape.getMesh();
    for(std::vector<Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
        mesh_.push_back(geo::Triangle(pose * it->p1_, pose * it->p2_, pose * it->p3_));
    }
}

}
