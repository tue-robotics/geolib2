#ifndef GEOLIB_OCTREENODE_H_
#define GEOLIB_OCTREENODE_H_

#include "datatypes.h"

namespace geo {

class Octree;

class OctreeNode {

public:

    OctreeNode(double size, Octree* octree);

    OctreeNode(const OctreeNode& orig, Octree* tree);

    virtual ~OctreeNode();

    OctreeNode* clone(Octree* tree) const;

    void add(const Vector3& p);

    void getCubes(std::vector<Box>& cubes, const Vector3& offset) const;

    bool intersect(const Ray& r, float t0, float t1, double& distance, const Vector3& offset) const;

    void raytrace(const Vector3& o, const Vector3& dir, float t0, float t1, const Vector3& offset);

    bool intersect(const Vector3& p) const;

    bool intersect(const Box& b) const;

protected:

    double size_;

    double split_;

    OctreeNode* children_[8];

    bool occupied_;

    Octree* tree_;

};

}

#endif
