#include "geolib/CompositeShape.h"
#include "geolib/datatypes.h"
#include "geolib/Ray.h"
#include "geolib/Shape.h"
#include "geolib/Triangle.h"

#include <algorithm>
#include <cmath>
#include <console_bridge/console.h>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace geo
{

CompositeShape::CompositeShape() : max_radius_(0), min_(1e10, 1e10, 1e10), max_(-1e10, -1e10, -1e10), bb_(-min_, -max_)
{
}

CompositeShape::~CompositeShape() = default;

CompositeShape* CompositeShape::clone() const
{
    return new CompositeShape(*this);
}

bool CompositeShape::intersect(const Ray& r, float t0, float t1, double& distance) const
{
    if (!bb_.intersect(r, t0, t1, distance))
    {
        return false;
    }

    bool hit = false;
    double min_distance = t1;

    for (const auto& it : shapes_)
    {
        const Transform& pose_inv = it.second;

        const Shape& shape = *it.first;

        Ray const r_t(pose_inv * r.getOrigin(), pose_inv.getBasis() * r.getDirection());

        double d = NAN;
        if (shape.intersect(r_t, t0, static_cast<float>(min_distance), d))
        {
            min_distance = d;
            hit = true;
        }
    }

    if (hit)
    {
        distance = min_distance;
        return true;
    }

    return false;
}

bool CompositeShape::intersect(const Vector3& p, const double RADIUS) const
{
    if (!bb_.intersect(p, RADIUS))
    {
        return false;
    }
    return std::any_of(shapes_.begin(),
                       shapes_.end(),
                       [&p, RADIUS](const auto& shape)
                       {
                           const Transform& pose_inv = shape.second;
                           Vector3 const p_t = pose_inv * p;
                           return (shape.first)->intersect(p_t, RADIUS);
                       });
}

bool CompositeShape::contains(const Vector3& p) const
{
    if (!bb_.contains(p))
    {
        return false;
    }
    return std::any_of(shapes_.begin(),
                       shapes_.end(),
                       [&p](const auto& it)
                       {
                           const Transform& pose_inv = it.second;
                           const Shape& shape = *it.first;
                           Vector3 const p_t = pose_inv * p;
                           return shape.contains(p_t);
                       });
}

double CompositeShape::getMaxRadius() const
{
    return max_radius_;
}

void CompositeShape::addShape(const Shape& shape, const Pose3D& pose)
{
    // add to shapes
    shapes_.emplace_back(ShapePtr(shape.clone()), pose.inverse());

    // add to mesh
    const std::vector<Triangle>& triangles = shape.getMesh().getTriangles();
    for (const auto& triangle : triangles)
    {
        Vector3 const p1 = pose * triangle.p1();
        Vector3 const p2 = pose * triangle.p2();
        Vector3 const p3 = pose * triangle.p3();

        max_radius_ = std::max<double>(max_radius_, p1.length());
        max_radius_ = std::max<double>(max_radius_, p2.length());
        max_radius_ = std::max<double>(max_radius_, p3.length());

        min_.x = std::min<double>({min_.x, p1.x, p2.x, p3.x});
        min_.y = std::min<double>({min_.y, p1.y, p2.y, p3.y});
        min_.z = std::min<double>({min_.z, p1.z, p2.z, p3.z});

        max_.x = std::max<double>({max_.x, p1.x, p2.x, p3.x});
        max_.y = std::max<double>({max_.y, p1.y, p2.y, p3.y});
        max_.z = std::max<double>({max_.z, p1.z, p2.z, p3.z});
    }

    mesh_.add(shape.getMesh().getTransformed(pose));

    bb_ = Box(min_, max_);
}

Box CompositeShape::getBoundingBox() const
{
    return bb_;
}

const std::vector<std::pair<ShapePtr, Transform>>& CompositeShape::getShapes() const
{
    return shapes_;
}

void CompositeShape::setMesh(const Mesh& /*mesh*/)
{
    std::string const msg = "CompositeShape::setMesh: can not set mesh for CompositeShape";
    CONSOLE_BRIDGE_logError(msg.c_str());
    throw std::runtime_error(msg);
}

} // namespace geo
