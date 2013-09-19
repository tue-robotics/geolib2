#ifndef FAST_SIMULATOR_HEIGHT_MAP_H_
#define FAST_SIMULATOR_HEIGHT_MAP_H_

#include <tf/transform_datatypes.h>
#include "Ray.h"
#include "Box.h"

class HeightMap;

class HeightMapNode {

public:

    HeightMapNode(const Box& box);

    HeightMapNode(const HeightMapNode& orig);

    virtual ~HeightMapNode();

    HeightMapNode* clone() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    /*
    void add(const tf::Vector3& p);

    void getCubes(std::vector<Box>& cubes, const tf::Vector3& offset) const;



    bool intersect(const tf::Vector3& p) const;

    bool intersect(const Box& b) const;
    */

    Box box_;

    HeightMapNode* children_[4];

    bool occupied_;

};

#endif
