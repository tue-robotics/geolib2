#include "fast_simulator/HeightMap.h"

HeightMap::HeightMap() : root_(0) {
}

HeightMap::HeightMap(const HeightMap& orig) {
    if (orig.root_) {
        root_ = new HeightMapNode(*orig.root_);
    } else {
        root_ = 0;
    }
}

HeightMap::~HeightMap() {
    delete root_;
}

HeightMap* HeightMap::clone() const {
    return new HeightMap(*this);
}

bool HeightMap::intersect(const Ray& r, float t0, float t1, double& distance) const {
    if (!root_) {
        return false;
    }
    return root_->intersect(r, t0, t1, distance);
}

void HeightMap::getBoundingBox(tf::Vector3 &min, tf::Vector3 &max) const {
    if (root_) {
        min = root_->box_.bounds[0];
        max = root_->box_.bounds[1];
    } else {
        min = tf::Vector3(0, 0, 0);
        max = tf::Vector3(0, 0, 0);
    }
}

HeightMap HeightMap::fromGrid(const std::vector<std::vector<double> >& grid, double resolution) {
    unsigned int mx_max = grid.size();
    unsigned int my_max = grid[0].size();

    unsigned int size = std::max(mx_max, my_max);

    unsigned int pow_size = 1;
    while(pow_size < size) {
        pow_size *= 2;
    }

    std::vector<std::vector<double> > pow_grid(pow_size);

    for(unsigned int mx = 0; mx < mx_max; ++mx) {
        pow_grid[mx].resize(pow_size, 0);
        for(unsigned int my = 0; my < my_max; ++my) {
            pow_grid[mx][my] = grid[mx][my];
        }
    }

    for(unsigned int mx = mx_max; mx < pow_size; ++mx) {
         pow_grid[mx].resize(pow_size, 0);
    }

    HeightMap hmap;
    hmap.root_ = createQuadTree(pow_grid, 0, 0, pow_size, pow_size, resolution);
    return hmap;
}

HeightMapNode* HeightMap::createQuadTree(const std::vector<std::vector<double> >& map,
                            unsigned int mx_min, unsigned int my_min,
                            unsigned int mx_max, unsigned int my_max, double resolution) {

    //std::cout << mx_min << ", " << my_min << " - " << mx_max << ", " << my_max << std::endl;

    double max_height = 0;
    for(unsigned int mx = mx_min; mx < mx_max; ++mx) {
        for(unsigned int my = my_min; my < my_max; ++my) {
            max_height = std::max(max_height, map[mx][my]);
        }
    }

    if (max_height == 0) {
        return 0;
    }

    tf::Vector3 min_map((double)mx_min * resolution,
                        (double)my_min * resolution, 0);
    tf::Vector3 max_map((double)mx_max * resolution,
                        (double)my_max * resolution, max_height);

    HeightMapNode* n = new HeightMapNode(Box(min_map, max_map));

    if (mx_max - mx_min == 1 || my_max - my_min == 1) {
        assert(mx_max - mx_min == 1 && my_max - my_min == 1);
        n->occupied_ = true;
        return n;
    } else {
        n->occupied_ = false;

        unsigned int cx = (mx_max + mx_min) / 2;
        unsigned int cy = (my_max + my_min) / 2;

        n->children_[0] = createQuadTree(map, mx_min, my_min, cx, cy, resolution);
        n->children_[1] = createQuadTree(map, cx , my_min, mx_max, cy, resolution);
        n->children_[2] = createQuadTree(map, mx_min, cy , cx, my_max, resolution);
        n->children_[3] = createQuadTree(map, cx , cy , mx_max, my_max, resolution);

        /*
        if ((n->children_[0] && n->children_[0]->occupied_)
                || (n->children_[1] && n->children_[1]->occupied_)
                || (n->children_[2] && n->children_[2]->occupied_)
                || (n->children_[3] && n->children_[3]->occupied_)) {

            std::cout << min_map.getX() << ", " << min_map.getY() << ", " << min_map.getZ() << " - " << max_map.getX() << ", " << max_map.getY() << ", " << max_map.getZ() << std::endl;

            if (n->children_[0]) {
                std::cout << "    " << n->children_[0]->box_.bounds[0].getX() << ", " << n->children_[0]->box_.bounds[0].getY() << ", " << n->children_[0]->box_.bounds[0].getZ() << " - "
                                                                                                                                                                                  << n->children_[0]->box_.bounds[1].getX() << ", " << n->children_[0]->box_.bounds[1].getY() << ", " << n->children_[0]->box_.bounds[1].getZ() << std::endl;
            }

            if (n->children_[1]) {
                std::cout << "    " << n->children_[1]->box_.bounds[0].getX() << ", " << n->children_[1]->box_.bounds[0].getY() << ", " << n->children_[1]->box_.bounds[0].getZ() << " - "
                                                                                                                                                                                  << n->children_[1]->box_.bounds[1].getX() << ", " << n->children_[1]->box_.bounds[1].getY() << ", " << n->children_[1]->box_.bounds[1].getZ() << std::endl;
            }

            if (n->children_[2]) {
                std::cout << "    " << n->children_[2]->box_.bounds[0].getX() << ", " << n->children_[2]->box_.bounds[0].getY() << ", " << n->children_[2]->box_.bounds[0].getZ() << " - "
                                                                                                                                                                                  << n->children_[2]->box_.bounds[1].getX() << ", " << n->children_[2]->box_.bounds[1].getY() << ", " << n->children_[2]->box_.bounds[1].getZ() << std::endl;
            }

            if (n->children_[3]) {
                std::cout << "    " << n->children_[3]->box_.bounds[0].getX() << ", " << n->children_[3]->box_.bounds[0].getY() << ", " << n->children_[3]->box_.bounds[0].getZ() << " - "
                                                                                                                                                                                  << n->children_[3]->box_.bounds[1].getX() << ", " << n->children_[3]->box_.bounds[1].getY() << ", " << n->children_[3]->box_.bounds[1].getZ() << std::endl;
            }

        }
        */

    }

    return n;
}

//void HeightMap::add(double x, double y, double height) {
//    if (!root_) {
//        root_ = new HeightMapNode(Box(tf::Vector3(x, y, 0), tf::Vector3(x + resolution_, y + resolution_, height)));
//    } else {




//        bool resize = false;


//        if (x < root_->box_.bounds[0].getX() || x + resolution_ > root_->box_.bounds[1].getX()
//               || y < root_->box_.bounds[0].getY() || y + resolution_ > root_->box_.bounds[1].getY()) {

//    }
//}


