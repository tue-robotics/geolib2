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
     * @brief Determines whether a point p lies within the shape
     * @param p point to test
     * @return True means point p lies inside the shape
     */
    virtual bool contains(const Vector3& p) const;

    virtual double getMaxRadius() const;

    virtual const Mesh& getMesh() const;

    virtual Box getBoundingBox() const;

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

    Mesh mesh_;

private:

    mutable bool  bounding_box_cache_valid_;    // keeps track if the cached values of the bounding box are valid
    mutable Vector3 bounding_box_min_cache_;    // cached value of the min corner of a bounding box
    mutable Vector3 bounding_box_max_cache_;    // cached value of the max corner of a bounding box
};

}


#endif
