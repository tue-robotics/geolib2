#ifndef GEOLIB_MESH_H_
#define GEOLIB_MESH_H_

#include "Triangle.h"

namespace geo {

struct Index3 {

    Index3(int i1, int i2, int i3) : i1_(i1), i2_(i2), i3_(i3) {}

    int i1_, i2_, i3_;
};

class Mesh {

public:

    Mesh();

    virtual ~Mesh();

    int addPoint(double x, double y, double z);

    void addTriangle(int i1, int i2, int i3);

    void add(const Mesh& mesh);

    inline bool empty() const { return triangles_i_.empty(); }

    inline std::size_t size() const { return triangles_i_.size(); }

    inline void clear() {
        triangles_i_.clear();
        points_.clear();
    }

    Mesh getTransformed(const tf::Transform t) const;

    const std::vector<Triangle>& getTriangles() const;

protected:

    std::vector<tf::Vector3> points_;

    std::vector<Index3> triangles_i_;

    mutable std::vector<Triangle> triangles_;

};

}


#endif
