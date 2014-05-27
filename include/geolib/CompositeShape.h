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

    double getMaxRadius() const;

    void addShape(const Shape& shape, const Pose3D& pose);

protected:

    std::vector<std::pair<ShapePtr, Transform> > shapes_;

    double max_radius_;

    Vector3 min_;

    Vector3 max_;

    Box bb_;

};

}


#endif
