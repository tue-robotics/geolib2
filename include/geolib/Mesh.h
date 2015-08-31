#ifndef GEOLIB_MESH_H_
#define GEOLIB_MESH_H_

#include "Triangle.h"

namespace geo {

/** Triangle mesh, as triplet of indices in the point vector. */
struct TriangleI {

    TriangleI(int i1, int i2, int i3) : i1_(i1), i2_(i2), i3_(i3) {}

    int i1_, i2_, i3_; ///< Points in the Mesh::points_ vector.
};

/** Mesh storage. */
class Mesh {

public:

    Mesh();

    virtual ~Mesh();

    int addPoint(double x, double y, double z);

    int addPoint(const geo::Vector3& p);

    void addTriangle(int i1, int i2, int i3);

    void add(const Mesh& mesh);

//    void removeTriangles(const std::set<int>& indices);

    void copyAndRemoveTriangles(const std::set<int>& indices, geo::Mesh& target) const;

    /**
     * Test whether the mesh is empty.
     * @return True if there are no triangles in the mesh (else false).
     */
    inline bool empty() const { return triangles_i_.empty(); }

    /**
     * Get the number of triangles of the mesh.
     * @return The number of triangles in the mesh.
     */
    inline std::size_t size() const { return triangles_i_.size(); }

    /** Empty the mesh. */
    inline void clear() {
        triangles_i_.clear();
        points_.clear();
    }

    Mesh getTransformed(const geo::Transform t) const;

    const std::vector<Triangle>& getTriangles() const;

    const std::vector<geo::Vector3>& getPoints() const;

    const std::vector<TriangleI>& getTriangleIs() const;

    void filterOverlappingVertices();

    double getMaxRadius() const;

protected:

    mutable double max_radius_; ///< Cached maximum radius.

    std::vector<geo::Vector3> points_; ///< Points of the mesh.

    std::vector<TriangleI> triangles_i_; ///< Triangles of the mesh.

    mutable std::vector<Triangle> triangles_; ///< Cached output result.

    inline int foo(int i, const std::vector<geo::Vector3>& vertices, std::vector<int>& vertex_mapping, std::vector<geo::Vector3>& vertices_new) const {
        if (vertex_mapping[i] >= 0) {
            return vertex_mapping[i];
        } else {
            int i_new = vertices_new.size();
            vertices_new.push_back(vertices[i]);
            vertex_mapping[i] = i_new;
            return i_new;
        }
    }

};

}


#endif
