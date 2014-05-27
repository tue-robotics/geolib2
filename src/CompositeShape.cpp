#include "geolib/CompositeShape.h"

namespace geo {

CompositeShape::CompositeShape() : max_radius_(0), min_(1e10, 1e10, 1e10), max_(-1e10, -1e10, -1e10), bb_(-min_, -max_) {
}

CompositeShape::~CompositeShape() {
}

CompositeShape* CompositeShape::clone() const {
    return new CompositeShape(*this);
}

bool CompositeShape::intersect(const Ray& r, float t0, float t1, double& distance) const {
    if (!bb_.intersect(r, t0, t1, distance)) {
        return false;
    }

    bool hit = false;
    double min_distance = t1;


    for(std::vector<std::pair<ShapePtr, Transform> >::const_iterator it = shapes_.begin(); it != shapes_.end(); ++it) {
        const Transform& pose_inv = it->second;

        const Shape& shape = *it->first;

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

double CompositeShape::getMaxRadius() const {
    return max_radius_;
}

void CompositeShape::addShape(const Shape& shape, const Pose3D& pose) {
    // add to shapes
    shapes_.push_back(std::pair<ShapePtr, Transform>(ShapePtr(shape.clone()), pose.inverse()));

    // add to mesh
    const std::vector<Triangle>& triangles = shape.getMesh().getTriangles();
    for(std::vector<Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
        Vector3 p1 = pose * it->p1_;
        Vector3 p2 = pose * it->p2_;
        Vector3 p3 = pose * it->p3_;

        max_radius_ = std::max(max_radius_, (double)p1.length());
        max_radius_ = std::max(max_radius_, (double)p2.length());
        max_radius_ = std::max(max_radius_, (double)p3.length());

        min_.x = std::min(min_.getX(), std::min(p1.x, std::min(p2.x, p3.x)));
        min_.y = std::min(min_.getY(), std::min(p1.y, std::min(p2.y, p3.y)));
        min_.z = std::min(min_.getZ(), std::min(p1.z, std::min(p2.z, p3.z)));

        max_.x = std::max(max_.getX(), std::max(p1.x, std::max(p2.x, p3.x)));
        max_.y = std::max(max_.getY(), std::max(p1.y, std::max(p2.y, p3.y)));
        max_.z = std::max(max_.getZ(), std::max(p1.z, std::max(p2.z, p3.z)));
    }

    mesh_.add(shape.getMesh().getTransformed(pose));

    bb_ = Box(min_, max_);
}

}
