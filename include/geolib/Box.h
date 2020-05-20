#ifndef GEOLIB_BOX_H_
#define GEOLIB_BOX_H_

#include "Shape.h"

namespace geo {

class Box : public Shape {

public:

    Box(const Vector3 &min, const Vector3 &max);

    Box* clone() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    /**
     * @brief Determines of this Box intersects with an other box
     * @param other second box
     * @return True means this box intersects with the other box
     */
    bool intersect(const Box& other) const;

    /** @brief Box::intersect() determines whether the shape intersects a sphere with center p.
     *  @return bool True means the sphere intersects the shape.
     **/
    bool intersect(const Vector3& p, const double radius) const;

    /** Deprecated! use Box::contains(p) instead!
     **/
    bool intersect(const Vector3& p) const;

    /** @brief Box::contains() determines whether a point p lies within the shape.
     *  @return bool True means point p lies inside the shape.
     **/
    bool contains(const Vector3& p) const;

    double getMaxRadius() const;

    Box getBoundingBox() const;

    void enclose(const Box& box, const Pose3D& pose);

    Vector3 getSize() const;

    Vector3 getCenter() const;

    Vector3 getMin() const;

    Vector3 getMax() const;

    Vector3 bounds[2];

protected:

    double max_radius_;

};

}

#endif
