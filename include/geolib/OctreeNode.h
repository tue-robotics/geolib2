#ifndef GEOLIB_OCTREENODE_H_
#define GEOLIB_OCTREENODE_H_

#include "datatypes.h"

#include <vector>

namespace geo
{

class Octree;

class OctreeNode
{

public:
    OctreeNode(double size, Octree* octree);

    OctreeNode(const OctreeNode& orig, Octree* tree);

    /**
     * @brief A node owns its children and refers to its tree, so it can only be copied for a tree, using the
     * constructor above
     */
    OctreeNode(const OctreeNode& orig) = delete;
    OctreeNode& operator=(const OctreeNode& rhs) = delete;

    virtual ~OctreeNode();

    OctreeNode* clone(Octree* tree) const;

    void add(const Vector3& p);

    void getCubes(std::vector<Box>& cubes, const Vector3& offset) const;

    bool intersect(const Ray& r, float t0, float t1, double& distance, const Vector3& offset) const;

    void raytrace(const Vector3& o, const Vector3& dir, float t0, float t1, const Vector3& offset);

    bool contains(const Vector3& p) const;

    bool intersect(const Box& b) const;

    /**
     * @brief Determines whether a sphere intersects the occupied space of this node. The sphere is assumed to intersect
     * the cube of this node.
     * @param p center of the sphere, relative to the minimum corner of this node
     * @param radius radius of the sphere
     * @return True means the sphere intersects
     */
    bool intersect(const Vector3& p, double radius) const;

    /**
     * @brief Extend the axis aligned box [min, max] such that it includes the occupied space of this node
     * @param min minimum corner of the box
     * @param max maximum corner of the box
     * @param offset position of the minimum corner of this node
     */
    void extendBoundingBox(Vector3& min, Vector3& max, const Vector3& offset) const;

protected:
    double size_;

    double split_;

    OctreeNode* children_[8]{};

    bool occupied_;

    Octree* tree_;
};

} // namespace geo

#endif
