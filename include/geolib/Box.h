#ifndef GEOLIB_BOX_H_
#define GEOLIB_BOX_H_

#include "Shape.h"

namespace geo {

/**
 * A class describing a box geometry
 *
 * The box is defined by two vectors. Its origin is therefore not in the center
 * of the box
 *
 */
class Box : public Shape {

public:

    Box(const Vector3 &min, const Vector3 &max);

    Box* clone() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    /**
     * @brief Determines whether this Box intersects with another box
     *
     * The two boxes are not rotated, they are assumed to be axis aligned.
     * @param other: second box
     * @return True means this box intersects with the other box
     */
    bool intersect(const Box& other) const;

    bool intersect(const Vector3& p, const double radius) const;

    bool contains(const Vector3& p) const;

    double getMaxRadius() const;

    /**
     * @copybrief Shape::getBoundingBox()
     * @brief Return a bounding box, which is the same as the object itself.
     * @return itself
     */
    Box getBoundingBox() const;

    void enclose(const Box& box, const Pose3D& pose);

    /**
     * @brief get the size of the box along all axes
     * @return vector with the sizes of the box along the corresponding axes
     */
    Vector3 getSize() const;

    /**
     * @brief Determine the center of the box with respect to the origin of the box.
     * @return
     */
    Vector3 getCenter() const;

    /**
     * @brief get vertex of the box with minimum coordinates
     * @return vector
     */
    const Vector3& getMin() const;

    /**
     * @brief get vertex of the box with maximum coordinates
     * @return vector
     */
    const Vector3& getMax() const;

    void setMesh(const Mesh& mesh);

protected:

    Vector3 bounds[2];

    double max_radius_;

    /**
     * @brief Should be called any time #bounds is changed
     */
    void generate_mesh_();

};

}

#endif
