#ifndef GEOLIB_COMPOSITE_SHAPE_H_
#define GEOLIB_COMPOSITE_SHAPE_H_

#include "Shape.h"
#include "geolib/Box.h"

#include <vector>

namespace geo {

class Box;

class CompositeShape : public Shape {

public:

    CompositeShape();

    virtual ~CompositeShape();

    CompositeShape* clone() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    /**
     * @brief Determines whether the shape intersects a sphere with center p
     * @param p center of the sphere
     * @param radius radius of the sphere
     * @return True means the sphere intersects the shape
     */
    bool intersect(const Vector3& p, const double radius) const;

    /**
     * @brief Determines whether a point p lies within the shape
     * @param p point to test
     * @return True means point p lies inside the shape
     */
    bool contains(const Vector3& p) const;

    double getMaxRadius() const;

    void addShape(const Shape& shape, const Pose3D& pose);

    Box getBoundingBox() const;

    const std::vector<std::pair<ShapePtr, Transform> >& getShapes() const;

    void setMesh(const Mesh &mesh);


protected:

    std::vector<std::pair<ShapePtr, Transform> > shapes_;

    double max_radius_;

    Vector3 min_;

    Vector3 max_;

    Box bb_;

};

}


#endif
