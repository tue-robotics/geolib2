#include "geolib/Box.h"
#include "geolib/datatypes.h"
#include "geolib/math_types.h"
#include "geolib/Ray.h"

#include <algorithm>
#include <array>
#include <console_bridge/console.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <vector>

namespace geo
{

Box::Box(const Vector3& min, const Vector3& max)
{
    bounds[0] = min;
    bounds[1] = max;
    generate_mesh_();
}

Box* Box::clone() const
{
    return new Box(*this);
}

bool Box::intersect(const Ray& r, float t0, float t1, double& distance) const
{
    // Slab test
    const geo::Vec3& origin = r.getOrigin();
    const geo::Vec3& inv_direction = r.getInvDirection();
    const std::array<int, 3>& sign = r.getSign();

    double tmin = -std::numeric_limits<double>::infinity();
    double tmax = std::numeric_limits<double>::infinity();
    for (uint i = 0; i < 3; ++i)
    {
        if (std::isinf(inv_direction[i]))
        {
            // Ray is parallel to this slab, so it can only hit the box if the origin lies within the slab. This is
            // checked explicitly, as 0 * inf is undefined.
            if (origin[i] < bounds[0][i] || origin[i] > bounds[1][i])
                return false;
            continue;
        }

        tmin = std::max(tmin, (bounds[sign[i]][i] - origin[i]) * inv_direction[i]);
        tmax = std::min(tmax, (bounds[1 - sign[i]][i] - origin[i]) * inv_direction[i]);
        if (tmin > tmax)
            return false;
    }

    distance = tmin;
    return t0 < tmax && tmin < t1;
}

double Box::getMaxRadius() const
{
    // The farthest point of the box is the corner with, per axis, the bound farthest from the origin
    const Vector3& min = getMin();
    const Vector3& max = getMax();
    return Vector3(std::max(std::abs(min.x), std::abs(max.x)),
                   std::max(std::abs(min.y), std::abs(max.y)),
                   std::max(std::abs(min.z), std::abs(max.z)))
        .length();
}

bool Box::intersect(const Box& other) const
{
    const Vector3& c1 = getCenter();
    const Vector3& c2 = other.getCenter();

    const Vector3& r1 = getSize() * 0.5;
    const Vector3& r2 = other.getSize() * 0.5;

    if (std::abs(c1.x - c2.x) > (r1.x + r2.x))
        return false;
    if (std::abs(c1.y - c2.y) > (r1.y + r2.y))
        return false;
    if (std::abs(c1.z - c2.z) > (r1.z + r2.z))
        return false;

    return true;
}

bool Box::intersect(const Vector3& p, const double RADIUS) const
{
    Vector3 c = getCenter();
    if (RADIUS <= 0)
    {
        return contains(p);
    }

    for (uint i = 0; i < 3; ++i)
    {
        if (p[i] > bounds[0][i] && p[i] < bounds[1][i])
            c[i] = p[i]; // If p is inside both bounds for this axis
        else
            c[i] = bounds[p[i] > c[i]][i]; // If p bigger than center take upper bound, otherwise lower bound
    }

    return RADIUS * RADIUS >= (p - c).length2();
}

bool Box::contains(const Vector3& p) const
{
    return (p.x >= bounds[0].x && p.x <= bounds[1].x && p.y >= bounds[0].y && p.y <= bounds[1].y &&
            p.z >= bounds[0].z && p.z <= bounds[1].z);
}

Box Box::getBoundingBox() const
{
    return *this;
}

void Box::enclose(const Box& box, const Pose3D& pose)
{
    const std::vector<Vector3> points = box.getMesh().getTransformed(pose).getPoints();

    for (const auto& point : points)
    {
        bounds[0].x = std::min<double>(bounds[0].x, point.x);
        bounds[0].y = std::min<double>(bounds[0].y, point.y);
        bounds[0].z = std::min<double>(bounds[0].z, point.z);

        bounds[1].x = std::max<double>(bounds[1].x, point.x);
        bounds[1].y = std::max<double>(bounds[1].y, point.y);
        bounds[1].z = std::max<double>(bounds[1].z, point.z);
    }
    generate_mesh_();
}

Vector3 Box::getSize() const
{
    return bounds[1] - bounds[0];
}

Vector3 Box::getCenter() const
{
    return (bounds[0] + bounds[1]) / 2;
}

const Vector3& Box::getMin() const
{
    return bounds[0];
}

const Vector3& Box::getMax() const
{
    return bounds[1];
}

void Box::setMesh(const Mesh& /*mesh*/)
{
    std::string const msg = "Box::setMesh: can not set mesh for Box";
    CONSOLE_BRIDGE_logError(msg.c_str());
    throw std::runtime_error(msg);
}

void Box::generate_mesh_()
{
    const Vector3& min = getMin();
    const Vector3& max = getMax();

    mesh_.clear();

    unsigned int const p0 = mesh_.addPoint(min.x, min.y, min.z); // 0
    unsigned int const p1 = mesh_.addPoint(max.x, min.y, min.z); // 1
    unsigned int const p2 = mesh_.addPoint(min.x, max.y, min.z); // 2
    unsigned int const p3 = mesh_.addPoint(max.x, max.y, min.z); // 3
    unsigned int const p4 = mesh_.addPoint(min.x, min.y, max.z); // 4
    unsigned int const p5 = mesh_.addPoint(max.x, min.y, max.z); // 5
    unsigned int const p6 = mesh_.addPoint(min.x, max.y, max.z); // 6
    unsigned int const p7 = mesh_.addPoint(max.x, max.y, max.z); // 7

    // back plane
    mesh_.addTriangle(p1, p0, p2);
    mesh_.addTriangle(p1, p2, p3);

    // front plane
    mesh_.addTriangle(p4, p5, p6);
    mesh_.addTriangle(p6, p5, p7);

    // left plane
    mesh_.addTriangle(p0, p4, p2);
    mesh_.addTriangle(p2, p4, p6);

    // right plane
    mesh_.addTriangle(p5, p1, p3);
    mesh_.addTriangle(p5, p3, p7);

    // top plane
    mesh_.addTriangle(p0, p1, p4);
    mesh_.addTriangle(p4, p1, p5);

    // bottom plane
    mesh_.addTriangle(p3, p2, p6);
    mesh_.addTriangle(p3, p6, p7);
}

} // namespace geo
