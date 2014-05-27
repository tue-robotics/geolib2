#ifndef FAST_SIMULATOR_HEIGHT_MAP_H_
#define FAST_SIMULATOR_HEIGHT_MAP_H_

#include "Ray.h"
#include "Box.h"

namespace geo {

class HeightMap;

class HeightMapNode {

public:

    HeightMapNode(const Box& box);

    HeightMapNode(const HeightMapNode& orig);

    virtual ~HeightMapNode();

    HeightMapNode* clone() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    Box box_;

    HeightMapNode* children_[4];

    bool occupied_;

};

}

#endif
