#ifndef GEOLIB_COMPOSITE_SHAPE_H_
#define GEOLIB_COMPOSITE_SHAPE_H_

#include "Shape.h"
#include "geolib/Box.h"

#include <vector>

namespace geo {

class Box;

/**
 * @brief A geometric description of a shape as a union of other shapes
 */
class CompositeShape : public Shape {

public:

    CompositeShape();

    virtual ~CompositeShape();

    CompositeShape* clone() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    bool intersect(const Vector3& p, const double radius) const;

    bool contains(const Vector3& p) const;

    double getMaxRadius() const;

    /**
     * @brief add a shape to the composite
     * @param shape: child shape to be added
     * @param pose: pose of the origin of the child shape relative to the origin of the composite shape
     */
    void addShape(const Shape& shape, const Pose3D& pose);

    Box getBoundingBox() const;

    /**
     * @brief Get all the child shapes and their inverse pose relative to the "origin" of the CompositeShape.
     * @return reference to the vector of all ShapePtr and Transform
     */
    const std::vector<std::pair<ShapePtr, Transform> >& getShapes() const;

    void setMesh(const Mesh &mesh);

protected:

    /**
     * pairs of child shapes and the transform from the origin of the child shape
     * to the origin of the composite shape
     */
    std::vector<std::pair<ShapePtr, Transform> > shapes_;

    double max_radius_;

    Vector3 min_;

    Vector3 max_;

    Box bb_;

};

}


#endif
