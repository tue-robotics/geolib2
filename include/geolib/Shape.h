#ifndef GEOLIB_SHAPE_H_
#define GEOLIB_SHAPE_H_

#include "Ray.h"
#include "Triangle.h"

namespace geo {

class Box;

class Shape {

public:

    Shape();

    virtual ~Shape();

    virtual Shape* clone() const = 0;

    virtual bool intersect(const Ray &, float t0, float t1, double& distance) const = 0;

    virtual double getMaxRadius() const;

    virtual const std::vector<Triangle>& getMesh() const;

protected:

    std::vector<Triangle> mesh_;
};

}


#endif
