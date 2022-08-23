#ifndef GEOLIB_MESH_H_
#define GEOLIB_MESH_H_

#include "Triangle.h"

#include <vector>

namespace geo {

/** Triangle mesh, as triplet of indices in the point vector. */
struct TriangleI {

    TriangleI(unsigned int i1, unsigned int i2, unsigned int i3) : i1_(i1), i2_(i2), i3_(i3) {}

    unsigned int i1_, i2_, i3_; ///< Points in the Mesh::points_ vector.

    // serialize TriangleI to stream
    friend std::ostream& operator<< (std::ostream& out, const TriangleI& tri) {
        out << "[ " << tri.i1_ << " " << tri.i2_ << " " << tri.i3_ << " ]";
        return out;
    }
};

/** Mesh storage. */
class Mesh {

public:

    Mesh();

    virtual ~Mesh();

    /**
     * Add a point to the mesh.
     * @param x X coordinate of the point to add.
     * @param y Y coordinate of the point to add.
     * @param z Z coordinate of the point to add.
     * @return Index of the point.
     */
    unsigned int addPoint(double x, double y, double z);

    /**
     * Add a point to the mesh.
     * @param p Point to add.
     * @return Index of the point.
     */
    unsigned int addPoint(const geo::Vector3& p);

    /**
     * Add a triangle to the mesh.
     * @param i1 First point of the triangle to add.
     * @param i2 Second point of the triangle to add.
     * @param i3 Third point of the triangle to add.
     */
    void addTriangle(unsigned int i1, unsigned int i2, unsigned int i3);

    /**
     * Add a mesh to this mesh.
     * @param mesh Mesh to add.
     */
    void add(const Mesh& mesh);

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

        invalidateCache();
    }

    /**
     * Apply transformation to the mesh.
     * @param t Transformation to apply.
     * @return The transformed mesh.
     */
    Mesh getTransformed(const geo::Transform t) const;

    /**
     * Get the triangles of the mesh.
     * @return The triangles of the mesh with points in space.
     */
    const std::vector<Triangle>& getTriangles() const;

    /**
     * Get the points of the mesh.
     * @return The points of the mesh.
     */
    const std::vector<geo::Vector3>& getPoints() const;

    /**
     * Get the triangles of the mesh.
     * @return The triangles of the mesh, using indices into the vector returned by #getPoints..
     */
    const std::vector<TriangleI>& getTriangleIs() const;

    /** Filter overlapping vertices from the mesh. */
    void filterOverlappingVertices();

    /**
     * Get the maximum radius.
     * @deprecated Use #getSquaredMaxRadius instead, as it is cheaper.
     * @return The maximum radius of the mesh.
     */
    double getMaxRadius() const;

    /**
     * Get the squared maximum radius.
     * @return The squared maximum radius of the mesh.
     */
    double getSquaredMaxRadius() const;

    /**
     * @brief Calculates the nornaml of a triangle in the mesh
     * @param index Indez of the traingle
     * @return the calculated normal of the triangle
     */
    const geo::Vector3 getTriangleNormal(unsigned int index) const;

protected:

    mutable double max_radius_cache_; ///< Cached maximum radius.
    mutable double max_radius_squared_cache_; ///< Cached squared maximum radius.

    std::vector<geo::Vector3> points_; ///< Points of the mesh.

    std::vector<TriangleI> triangles_i_; ///< Triangles of the mesh.

    mutable std::vector<Triangle> triangles_cache_; ///< Cached output result.

    /** Clear cached computed results, as they have become invalid. */
    void invalidateCache() {
        triangles_cache_.clear();
        max_radius_squared_cache_ = 0;
        max_radius_cache_ = 0;
    }
};

}


#endif
