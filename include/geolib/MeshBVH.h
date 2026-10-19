#ifndef GEOLIB_MESH_BVH_H_
#define GEOLIB_MESH_BVH_H_

#include "datatypes.h"

#include <array>
#include <vector>

namespace geo
{

class Mesh;
class Ray;

/**
 * @brief Bounding volume hierarchy of axis aligned boxes over the triangles of a mesh, which accelerates ray, sphere
 * and point queries from O(N) to O(log N) in the number of triangles.
 *
 * The triangles are copied, so the hierarchy stays valid independently of the mesh it was built from. It is immutable
 * after construction, so it can be queried from multiple threads.
 */
class MeshBVH
{

public:
    /**
     * @brief Build the hierarchy over all triangles of a mesh
     * @param mesh mesh to build the hierarchy for
     */
    explicit MeshBVH(const Mesh& mesh);

    /**
     * @brief Nearest intersection of a ray with the triangles
     * @param r ray
     * @param t0 minimum distance along the ray
     * @param t1 maximum distance along the ray
     * @param distance distance along the ray to the nearest intersection in (t0, t1); only set on intersection
     * @return true when the ray intersects a triangle
     */
    bool intersect(const Ray& r, double t0, double t1, double& distance) const;

    /**
     * @brief Determines whether any of the triangles lies within a distance @b radius of point @b p
     * @param p center of the sphere
     * @param radius radius of the sphere
     * @return True means the sphere intersects the surface
     */
    bool intersectSurface(const Vector3& p, double radius) const;

    /**
     * @brief Determines whether a point lies within the mesh, i.e. whether the absolute generalized winding number of
     * the mesh around the point is larger than 0.5. Points on the surface are considered to be inside. For closed
     * meshes, the winding number is determined by counting signed ray crossings using the hierarchy. For open meshes,
     * the winding number is calculated from the solid angles of all triangles.
     * @param p point to test
     * @return True means point p lies inside the mesh
     */
    bool contains(const Vector3& p) const;

    /**
     * @brief Whether the mesh is closed, i.e. every edge is used equally often in both directions, independent of
     * (duplicated) vertex indices
     */
    bool isClosed() const { return closed_; }

    /**
     * @brief Whether there are no triangles
     */
    bool empty() const { return triangles_.empty(); }

    /**
     * @brief Number of nodes of the hierarchy
     */
    std::size_t numNodes() const { return nodes_.size(); }

    /**
     * @brief Maximum number of triangles in a leaf node
     */
    static constexpr unsigned int MAX_LEAF_SIZE = 4;

protected:
    /**
     * @brief Node of the hierarchy. A leaf contains the triangles [first, first + count). For an inner node, count is
     * zero, the first child directly follows the node and first is the index of the second child.
     */
    struct Node
    {
        Vector3 min;
        Vector3 max;
        unsigned int first;
        unsigned int count;
    };

    using Triangle = std::array<Vector3, 3>;

    std::vector<Node> nodes_;

    std::vector<Triangle> triangles_;

    bool closed_;

    unsigned int build(std::vector<unsigned int>& order,
                       const std::vector<Vector3>& centroids,
                       unsigned int begin,
                       unsigned int end);

    /**
     * @brief Count the signed crossings of the ray from @b p in direction @b dir with the triangles
     * @param p origin of the ray
     * @param dir direction of the ray
     * @param count sum of the signs of the crossings
     * @return false if the ray hits a triangle (almost) on an edge or vertex, or (almost) in its plane, in which case
     * the count is not reliable
     */
    bool countCrossings(const Vector3& p, const Vector3& dir, int& count) const;

    /**
     * @brief Absolute generalized winding number of all triangles around @b p, using the solid angles
     */
    double windingNumber(const Vector3& p) const;
};

} // namespace geo

#endif
