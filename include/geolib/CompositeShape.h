#ifndef GEOLIB_COMPOSITE_SHAPE_H_
#define GEOLIB_COMPOSITE_SHAPE_H_

#include "Shape.h"
#include "geolib/Box.h"

namespace geo {

class Box;

class CompositeShape : public Shape {

public:

    CompositeShape();

    virtual ~CompositeShape();

    CompositeShape* clone() const;

    bool intersect(const Ray &, float t0, float t1, double& distance) const;

    /** @brief CompositeShape::intersect() determines whether the shape intersects a sphere with center p.
     *  @return bool True means the sphere intersects the shape.
     **/
    bool intersect(const Vector3& p, const double radius) const;

    /** @brief CompositeShape::contains() determines whether a point p lies within the shape.
     *  @return bool True means point p lies inside the shape.
     **/
    bool contains(const Vector3& p) const;

    double getMaxRadius() const;

    void addShape(const Shape& shape, const Pose3D& pose);

    Box getBoundingBox() const;

    const std::vector<std::pair<ShapePtr, Transform> > &getShapes() const;


protected:

    std::vector<std::pair<ShapePtr, Transform> > shapes_;

    double max_radius_;

    Vector3 min_;

    Vector3 max_;

    Box bb_;

};

}


#endif
