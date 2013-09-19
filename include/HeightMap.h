#ifndef _HeightMap_H_
#define _HeightMap_H_

#include "Shape.h"
#include "HeightMapNode.h"
#include "Box.h"

class HeightMap : public Shape {

public:

    HeightMap();

    HeightMap(const HeightMap& orig);

    virtual ~HeightMap();

    HeightMap* clone() const;

    bool intersect(const Ray &, float t0, float t1, double& distance) const;

    void getBoundingBox(tf::Vector3 &min, tf::Vector3 &max) const;

    void add(double x, double y, double height);

    static HeightMap fromGrid(const std::vector<std::vector<double> >& grid, double resolution);

protected:

    //double resolution_;

    HeightMapNode* root_;

    static HeightMapNode* createQuadTree(const std::vector<std::vector<double> >& map,
                                unsigned int mx_min, unsigned int my_min,
                                unsigned int mx_max, unsigned int my_max, double resolution);

};

#endif
