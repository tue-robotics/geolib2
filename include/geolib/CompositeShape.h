#ifndef GEOLIB_COMPOSITE_SHAPE_H_
#define GEOLIB_COMPOSITE_SHAPE_H_

#include "Shape.h"

namespace geo {

class Box;

class CompositeShape : public Shape {

public:

    CompositeShape();

    virtual ~CompositeShape();

    CompositeShape* clone() const;

    bool intersect(const Ray &, float t0, float t1, double& distance) const;

    void addShape(const Shape& shape, const Pose3D& pose);

protected:

    std::vector<std::pair<ShapePtr, tf::Transform> > shapes_;

};

}


#endif
