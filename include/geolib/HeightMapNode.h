#ifndef GEOLIB_HEIGHT_MAP_NODE_H_
#define GEOLIB_HEIGHT_MAP_NODE_H_

#include "Box.h"
#include "Ray.h"

namespace geo
{

class HeightMap;

/**
 * One node of the quad tree used to make the heightmap.
 *
 * A leaf node describes a box of occupied space
 * A branch node describes geometry as the union of the geometry of its child nodes.
 */
class HeightMapNode
{

public:
    HeightMapNode(Box box);

    HeightMapNode(const HeightMapNode& orig);

    HeightMapNode(HeightMapNode&& orig) noexcept;

    HeightMapNode& operator=(const HeightMapNode& rhs);

    virtual ~HeightMapNode();

    HeightMapNode* clone() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    /**
     * @brief Determines whether a sphere intersects the occupied space of this node
     * @param p center of the sphere
     * @param radius radius of the sphere
     * @return True means the sphere intersects
     */
    bool intersect(const Vector3& p, double radius) const;

    /**
     * @brief Determines whether a point lies within the occupied space of this node
     * @param p point to test
     * @return True means point p lies inside
     */
    bool contains(const Vector3& p) const;

    // the bounding box of the node
    Box box_;

    // child nodes. In case the node is a leaf of the tree all four will be nullptr
    HeightMapNode* children_[4]{};

    // True iff the entire volume described by the bounding box is occupied
    bool occupied_;
};

} // namespace geo

#endif
