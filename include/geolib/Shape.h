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

    virtual double getMaxRadius() const;

    virtual const Mesh& getMesh() const;

    virtual const Box getBoundingBox() const;

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

    mutable bool  bounding_box_cache_valid_;
    mutable Vector3 bounding_box_min_cache_;
    mutable Vector3 bounding_box_max_cache_;

};

}


#endif
