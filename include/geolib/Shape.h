#ifndef GEOLIB_SHAPE_H_
#define GEOLIB_SHAPE_H_

#include "Ray.h"
#include "Triangle.h"
#include "Mesh.h"
#include "datatypes.h"

#include <boost/archive/basic_binary_iarchive.hpp>

namespace geo {

class Box;

class Shape {

    friend class Importer;

public:

    Shape();

    virtual ~Shape();

    virtual Shape* clone() const;

    virtual bool intersect(const Ray &, float t0, float t1, double& distance) const;

    /** @brief Shape::intersect(p, radius) determines whether the shape intersects a sphere with center p.
     *  @return bool True means the sphere intersects the shape.
     **/
    virtual bool intersect(const Vector3& p, const double radius) const;

    /** @brief Shape::intersect(shape) determines whether the shape intersects another shape.
     *  @return bool True means the two shapes intersect.
     **/
    virtual bool intersect(const Pose3D& pose, const Shape& other) const;

    /** @brief Shape::contains() determines whether a point p lies within the shape.
     *  @return bool True means point p lies inside the shape.
     **/
    virtual bool contains(const Vector3& p) const;

    virtual double getMaxRadius() const;

    virtual const Mesh& getMesh() const;

    virtual Box getBoundingBox() const;

    void setMesh(const Mesh& mesh);

    virtual bool write(std::ostream& output) const;

    static ShapePtr read(std::istream& input);

    static const std::string TYPE;

    /**
     * @brief empty Test whether the shape(mesh) is empty.
     * @return True if the mesh is empty
     */
    inline virtual bool empty() const { return mesh_.empty(); }

protected:

    Mesh mesh_;

private:

    mutable bool  bounding_box_cache_valid_;    // keeps track if the cached values of the bounding box are valid
    mutable Vector3 bounding_box_min_cache_;    // cached value of the min corner of a bounding box
    mutable Vector3 bounding_box_max_cache_;    // cached value of the max corner of a bounding box
};

}


#endif
