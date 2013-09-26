#include "geolib/Box.h"

namespace geo {

Box::Box(const Vector3 &min, const Vector3 &max) {
    bounds[0] = min;
    bounds[1] = max;

    // back plane

    mesh_.push_back(Triangle(Vector3(min.x(), min.y(), min.z()),
                             Vector3(max.x(), min.y(), min.z()),
                             Vector3(min.x(), max.y(), min.z())));


    mesh_.push_back(Triangle(Vector3(max.x(), min.y(), min.z()),
                             Vector3(min.x(), max.y(), min.z()),
                             Vector3(max.x(), max.y(), min.z())));

    // front plane

    mesh_.push_back(Triangle(Vector3(min.x(), min.y(), max.z()),
                             Vector3(max.x(), min.y(), max.z()),
                             Vector3(min.x(), max.y(), max.z())));

    mesh_.push_back(Triangle(Vector3(max.x(), min.y(), max.z()),
                             Vector3(min.x(), max.y(), max.z()),
                             Vector3(max.x(), max.y(), max.z())));

    // left plane

    mesh_.push_back(Triangle(Vector3(min.x(), min.y(), min.z()),
                             Vector3(min.x(), min.y(), max.z()),
                             Vector3(min.x(), max.y(), min.z())));

    mesh_.push_back(Triangle(Vector3(min.x(), min.y(), max.z()),
                             Vector3(min.x(), max.y(), min.z()),
                             Vector3(min.x(), max.y(), max.z())));

    // right plane

    mesh_.push_back(Triangle(Vector3(max.x(), min.y(), min.z()),
                             Vector3(max.x(), min.y(), max.z()),
                             Vector3(max.x(), max.y(), min.z())));

    mesh_.push_back(Triangle(Vector3(max.x(), min.y(), max.z()),
                             Vector3(max.x(), max.y(), min.z()),
                             Vector3(max.x(), max.y(), max.z())));

    // top plane

    mesh_.push_back(Triangle(Vector3(min.x(), min.y(), min.z()),
                             Vector3(max.x(), min.y(), min.z()),
                             Vector3(min.x(), min.y(), max.z())));

    mesh_.push_back(Triangle(Vector3(max.x(), min.y(), min.z()),
                             Vector3(min.x(), min.y(), max.z()),
                             Vector3(max.x(), min.y(), max.z())));



    // bottom plane

    mesh_.push_back(Triangle(Vector3(min.x(), max.y(), min.z()),
                             Vector3(max.x(), max.y(), min.z()),
                             Vector3(min.x(), max.y(), max.z())));

    mesh_.push_back(Triangle(Vector3(max.x(), max.y(), min.z()),
                             Vector3(min.x(), max.y(), max.z()),
                             Vector3(max.x(), max.y(), max.z())));


}

Box* Box::clone() const {
    return new Box(*this);
}

bool Box::intersect(const Ray &r, float t0, float t1, double& distance) const {

    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    tmin = (bounds[r.sign[0]].x() - r.origin_.x()) * r.inv_direction_.x();
    tmax = (bounds[1-r.sign[0]].x() - r.origin_.x()) * r.inv_direction_.x();
    tymin = (bounds[r.sign[1]].y() - r.origin_.y()) * r.inv_direction_.y();
    tymax = (bounds[1-r.sign[1]].y() - r.origin_.y()) * r.inv_direction_.y();

    if ( (tmin > tymax) || (tymin > tmax) )
        return false;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    tzmin = (bounds[r.sign[2]].z() - r.origin_.z()) * r.inv_direction_.z();
    tzmax = (bounds[1-r.sign[2]].z() - r.origin_.z()) * r.inv_direction_.z();
    if ( (tmin > tzmax) || (tzmin > tmax) )
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

    distance = tmin;
    return t0 < tmax && tmin < t1;
}

bool Box::intersect(const Box& other) const {
    Vector3 c1 = (bounds[0] + bounds[1]) / 2;
    Vector3 c2 = (other.bounds[0] + other.bounds[1]) / 2;

    Vector3 r1 = (bounds[1] - bounds[0]) / 2;
    Vector3 r2 = (other.bounds[1] - other.bounds[0]) / 2;

    if (std::abs(c1.getX() - c2.getX()) > r1.getX() + r2.getX()) return false;
    if (std::abs(c1.getY() - c2.getY()) > r1.getY() + r2.getY()) return false;
    if (std::abs(c1.getZ() - c2.getZ()) > r1.getZ() + r2.getZ()) return false;

    return true;
}

bool Box::intersect(const Vector3& p) const {
    return (p.getX() > bounds[0].getX() && p.getX() < bounds[1].getX()
            && p.getY() > bounds[0].getY() && p.getZ() < bounds[1].getY()
            && p.getZ() > bounds[0].getZ() && p.getY() < bounds[1].getZ());
}

Box Box::getBoundingBox() const {
    return *this;
}

void Box::enclose(const Box& box, const Pose3D& pose) {
    const Vector3& a = box.bounds[0];
    const Vector3& b = box.bounds[1];

    std::vector<Vector3> points;
    points.push_back(pose * Vector3(a.getX(), a.getY(), a.getZ()));
    points.push_back(pose * Vector3(a.getX(), a.getY(), b.getZ()));
    points.push_back(pose * Vector3(a.getX(), b.getY(), a.getZ()));
    points.push_back(pose * Vector3(a.getX(), b.getY(), b.getZ()));
    points.push_back(pose * Vector3(b.getX(), a.getY(), a.getZ()));
    points.push_back(pose * Vector3(b.getX(), a.getY(), b.getZ()));
    points.push_back(pose * Vector3(b.getX(), b.getY(), a.getZ()));
    points.push_back(pose * Vector3(b.getX(), b.getY(), b.getZ()));

    for(unsigned int i = 0; i < 8; ++i) {
        //std::cout << points[i] << std::endl;
        bounds[0].setX(std::min(bounds[0].getX(), points[i].getX()));
        bounds[0].setY(std::min(bounds[0].getY(), points[i].getY()));
        bounds[0].setZ(std::min(bounds[0].getZ(), points[i].getZ()));

        bounds[1].setX(std::max(bounds[1].getX(), points[i].getX()));
        bounds[1].setY(std::max(bounds[1].getY(), points[i].getY()));
        bounds[1].setZ(std::max(bounds[1].getZ(), points[i].getZ()));
    }
    //std::cout << std::endl;
}

Vector3 Box::getSize() const {
    return tf::Vector3(bounds[1].x() - bounds[0].x(), bounds[1].y() - bounds[0].y(), bounds[1].z() - bounds[0].z());
}

Vector3 Box::getCenter() const {
    return (bounds[0] + bounds[1]) / 2;
}

}

