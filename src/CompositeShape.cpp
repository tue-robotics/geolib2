#include "geolib/CompositeShape.h"

#include <console_bridge/console.h>

#include <stdexcept>

namespace geo {

CompositeShape::CompositeShape() : Shape(), max_radius_(0), min_(1e10, 1e10, 1e10), max_(-1e10, -1e10, -1e10), bb_(-min_, -max_) {
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

        Ray r_t(pose_inv * r.getOrigin(), pose_inv.getBasis() * r.getDirection());

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

bool CompositeShape::intersect(const Vector3& p, const double radius) const {
    if (!bb_.intersect(p, radius)) {
        return false;
    }
    for(auto it = shapes_.begin(); it != shapes_.end(); ++it) {
        const Transform& pose_inv = it->second;
        Vector3 p_t = pose_inv * p;
        if ((it->first)->intersect(p_t, radius)) {
            return true;
        }
    }
    return false;
}

bool CompositeShape::contains(const Vector3& p) const {
    if (!bb_.contains(p)) {
        return false;
    }
    for(auto it = shapes_.begin(); it != shapes_.end(); ++it) {
        const Transform& pose_inv = it->second;

        const Shape& shape = *it->first;

        Vector3 p_t = pose_inv * p;

        if (shape.contains(p_t)) {
            return true;
        }
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
        Vector3 p1 = pose * it->p1();
        Vector3 p2 = pose * it->p2();
        Vector3 p3 = pose * it->p3();

        max_radius_ = std::max<double>(max_radius_, p1.length());
        max_radius_ = std::max<double>(max_radius_, p2.length());
        max_radius_ = std::max<double>(max_radius_, p3.length());

        min_.x = std::min<double>(min_.x, std::min(p1.x, std::min(p2.x, p3.x)));
        min_.y = std::min<double>(min_.y, std::min(p1.y, std::min(p2.y, p3.y)));
        min_.z = std::min<double>(min_.z, std::min(p1.z, std::min(p2.z, p3.z)));

        max_.x = std::max<double>(max_.x, std::max(p1.x, std::max(p2.x, p3.x)));
        max_.y = std::max<double>(max_.y, std::max(p1.y, std::max(p2.y, p3.y)));
        max_.z = std::max<double>(max_.z, std::max(p1.z, std::max(p2.z, p3.z)));
    }

    mesh_.add(shape.getMesh().getTransformed(pose));

    bb_ = Box(min_, max_);
}

Box CompositeShape::getBoundingBox() const {
    return bb_;
}

const std::vector<std::pair<ShapePtr, Transform> >& CompositeShape::getShapes() const {
    return shapes_;
}

void CompositeShape::setMesh(const Mesh& /*mesh*/) {
    std::string msg = "CompositeShape::setMesh: can not set mesh for CompositeShape";
    CONSOLE_BRIDGE_logError(msg.c_str());
    throw std::runtime_error(msg);
}

}
