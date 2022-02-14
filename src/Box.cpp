#include "geolib/Box.h"

#include <console_bridge/console.h>

#include <stdexcept>
#include <string>

namespace geo {

Box::Box(const Vector3 &min, const Vector3 &max) {
    bounds[0] = min;
    bounds[1] = max;
    generate_mesh_();
}

Box* Box::clone() const {
    return new Box(*this);
}

bool Box::intersect(const Ray& r, float t0, float t1, double& distance) const {

    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    tmin = (bounds[r.sign[0]].x - r.origin_.x) * r.inv_direction_.x;
    tmax = (bounds[1-r.sign[0]].x - r.origin_.x) * r.inv_direction_.x;
    tymin = (bounds[r.sign[1]].y - r.origin_.y) * r.inv_direction_.y;
    tymax = (bounds[1-r.sign[1]].y - r.origin_.y) * r.inv_direction_.y;

    if ( (tmin > tymax) || (tymin > tmax) )
        return false;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    tzmin = (bounds[r.sign[2]].z - r.origin_.z) * r.inv_direction_.z;
    tzmax = (bounds[1-r.sign[2]].z - r.origin_.z) * r.inv_direction_.z;
    if ( (tmin > tzmax) || (tzmin > tmax) )
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

    distance = tmin;
    return t0 < tmax && tmin < t1;
}

double Box::getMaxRadius() const {
    return std::max(getMin().length(), getMax().length());
}

bool Box::intersect(const Box& other) const {
    const Vector3& c1 = getCenter();
    const Vector3& c2 = other.getCenter();

    const Vector3& r1 = getSize() * 0.5;
    const Vector3& r2 = other.getSize() * 0.5;

    if (std::abs<double>(c1.x - c2.x) > (r1.x + r2.x)) return false;
    if (std::abs<double>(c1.y - c2.y) > (r1.y + r2.y)) return false;
    if (std::abs<double>(c1.y - c2.z) > (r1.z + r2.z)) return false;

    return true;
}

bool Box::intersect(const Vector3& p, const double radius) const {
    Vector3 c = getCenter();

    if (p.x > bounds[0].x && p.x < bounds[1].x)
        c.x = p.getX();
    else if (p.x > c.x)
        c.x = bounds[1].x;
    else
        c.x = bounds[0].x;

    if (p.y > bounds[0].y && p.y < bounds[1].y)
        c.y = p.y;
    else if (p.y > c.y)
        c.y = bounds[1].y;
    else
        c.y = bounds[0].y;

    if (p.z > bounds[0].z && p.z < bounds[1].z)
        c.z = p.z;
    else if (p.z > c.z)
        c.z = bounds[1].z;
    else
        c.z = bounds[0].z;

    return radius*radius > (p-c).length2();
}

bool Box::contains(const Vector3& p) const {
    return (p.x > bounds[0].x && p.x < bounds[1].x
            && p.y > bounds[0].y && p.y < bounds[1].y
            && p.z > bounds[0].z && p.z < bounds[1].z);
}

Box Box::getBoundingBox() const {
    return *this;
}

void Box::enclose(const Box& box, const Pose3D& pose) {
    const std::vector<Vector3> points = box.getMesh().getTransformed(pose).getPoints();

    for(auto it = points.cbegin(); it != points.cend(); ++it) {
        bounds[0].x = std::min<double>(bounds[0].x, it->x);
        bounds[0].y = std::min<double>(bounds[0].y, it->y);
        bounds[0].z = std::min<double>(bounds[0].z, it->z);

        bounds[1].x = std::max<double>(bounds[1].x, it->x);
        bounds[1].y = std::max<double>(bounds[1].y, it->y);
        bounds[1].z = std::max<double>(bounds[1].z, it->z);
    }
    generate_mesh_();
}

Vector3 Box::getSize() const {
    return bounds[1] - bounds[0];
}

Vector3 Box::getCenter() const {
    return (bounds[0] + bounds[1]) / 2;
}

const Vector3& Box::getMin() const {
    return bounds[0];
}

const Vector3& Box::getMax() const {
    return bounds[1];
}

void Box::setMesh(const Mesh& /*mesh*/) {
    std::string msg = "Box::setMesh: can not set mesh for Box";
    CONSOLE_BRIDGE_logError(msg.c_str());
    throw std::runtime_error(msg);
}

void Box::generate_mesh_() {
    const Vector3& min = getMin();
    const Vector3& max = getMax();

    unsigned int p0 = mesh_.addPoint(min.x, min.y, min.z); // 0
    unsigned int p1 = mesh_.addPoint(max.x, min.y, min.z); // 1
    unsigned int p2 = mesh_.addPoint(min.x, max.y, min.z); // 2
    unsigned int p3 = mesh_.addPoint(max.x, max.y, min.z); // 3
    unsigned int p4 = mesh_.addPoint(min.x, min.y, max.z); // 4
    unsigned int p5 = mesh_.addPoint(max.x, min.y, max.z); // 5
    unsigned int p6 = mesh_.addPoint(min.x, max.y, max.z); // 6
    unsigned int p7 = mesh_.addPoint(max.x, max.y, max.z); // 7

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

}

