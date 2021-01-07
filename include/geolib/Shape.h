#ifndef GEOLIB_SHAPE_H_
#define GEOLIB_SHAPE_H_

#include "datatypes.h"
#include "Mesh.h"
#include "Ray.h"
#include "Triangle.h"

namespace geo {

class Box;

class Shape {

    friend class Importer;

public:

    Shape();

    virtual ~Shape();

    virtual Shape* clone() const;

    virtual bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    /**
     * @brief Determines whether the shape intersects a sphere with center p
     * @param p center of the sphere
     * @param radius radius of the sphere
     * @return True means the sphere intersects the shape
     */
    virtual bool intersect(const Vector3& p, const double radius) const;

    /**
     * @brief Shape::intersect(pose, shape) determines whether the shape intersects another shape.
     * @param Pose The position of the frame of shape self represented in the frame of shape other.
     * @param other Other shape to check intersection with.
     * @return True means the two shapes intersect.
     **/
    virtual bool intersect(const Pose3D& pose, const Shape& other) const;

    /**
     * @brief Determines whether a point p lies within the shape
     * @param p point to test
     * @return True means the point lies inside the shape
     */
    virtual bool contains(const Vector3& p) const;

    virtual double getMaxRadius() const;

    virtual Box getBoundingBox() const;

    virtual const Mesh& getMesh() const;

    /**
     * @brief set the Mesh
     * Any child classes should throw a std::logic_error in case the mesh should not be changed via #setMesh.
     * @param mesh mesh to set
     */
    virtual void setMesh(const Mesh& mesh);

    virtual bool write(std::ostream& output) const;

    static ShapePtr read(std::istream& input);

    static const std::string TYPE;

    /**
     * @brief empty Test whether the shape(mesh) is empty.
     * @return True if the mesh is empty
     */
    inline virtual bool empty() const { return mesh_.empty(); }

protected:

    /**
     * @brief Should not be read or written to directly in general. Use #setMesh and #getMesh to write respectively read the mesh.
     * In a few exceptions, the mesh can be written direcly. Make sure that mesh keeps consistent with other member variables.
     */
    Mesh mesh_;

private:

    mutable bool  bounding_box_cache_valid_;    // keeps track if the cached values of the bounding box are valid
    mutable Vector3 bounding_box_min_cache_;    // cached value of the min corner of a bounding box
    mutable Vector3 bounding_box_max_cache_;    // cached value of the max corner of a bounding box
};

}


#endif
