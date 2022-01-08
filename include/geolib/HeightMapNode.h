#ifndef GEOLIB_HEIGHT_MAP_NODE_H_
#define GEOLIB_HEIGHT_MAP_NODE_H_

#include "Ray.h"
#include "Box.h"

namespace geo {

class HeightMap;

/**
 * One node of the quad tree used to make the heightmap.
 *
 * A leaf node describes a box of occupied space
 * A branch node describes geometry as the union of the geometry of its child nodes.
 */
class HeightMapNode {

public:

    HeightMapNode(const Box& box);

    HeightMapNode(const HeightMapNode& orig);

    HeightMapNode(HeightMapNode&& orig);

    virtual ~HeightMapNode();

    HeightMapNode* clone() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    // the bounding box of the node
    Box box_;

    // child nodes. In case the node is a leaf of the tree all four will be nullptr
    HeightMapNode* children_[4];

    // True iff the entire volume described by the bounding box is occupied
    bool occupied_;

};

}

#endif
