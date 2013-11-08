#ifndef GEOLIB_BOX_H_
#define GEOLIB_BOX_H_

#include "Shape.h"

namespace geo {

class Box : public Shape {

public:

    Box(const Vector3 &min, const Vector3 &max);

    Box* clone() const;

    bool intersect(const Ray &, float t0, float t1, double& distance) const;

    double getMaxRadius() const;

    bool intersect(const Box& other) const;

    bool intersect(const Vector3& p) const;

    Box getBoundingBox() const;

    void enclose(const Box& box, const Pose3D& pose);

    Vector3 getSize() const;

    Vector3 getCenter() const;

    Vector3 bounds[2];

protected:

    double max_radius_;

};

}

#endif
