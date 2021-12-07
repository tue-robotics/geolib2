#ifndef GEOLIB_HEIGHT_MAP_H_
#define GEOLIB_HEIGHT_MAP_H_

#include "Shape.h"
#include "HeightMapNode.h"
#include "Box.h"

#include <vector>

namespace geo {

/**
 * @brief A geometric description of a Heightmap using a quad tree
 */
class HeightMap : public Shape {

public:

    HeightMap();

    HeightMap(const HeightMap& orig);

    virtual ~HeightMap();

    HeightMap* clone() const;

    bool intersect(const Ray &, float t0, float t1, double& distance) const;

    /**
     * @brief fromGrid: instantiate a Heightmap from a grid
     * @param grid: Heightmap in the form of a grid
     * @param resolution: resolution of the grid in meters per index
     * @return Heightmap
     */
    static HeightMap fromGrid(const std::vector<std::vector<double> >& grid, double resolution);

protected:

    //double resolution_;

    HeightMapNode* root_;

    /**
     * @brief createQuadTree: divide a grid over a quad tree
     * @param map: heightmap in the form of a grid. Must be square.
     * @param mx_min: indices describing a square region in the map from which to create the node
     * @param my_min: index to describe the region in the map
     * @param mx_max: index to describe the region in the map
     * @param my_max: index to describe the region in the map
     * @param resolution: resolution of the map in meters per index.
     * @return pointer to the root node of the quadtree.
     */
    static HeightMapNode* createQuadTree(const std::vector<std::vector<double> >& map,
                                unsigned int mx_min, unsigned int my_min,
                                unsigned int mx_max, unsigned int my_max, double resolution);

};

}

#endif
