#include "geolib/HeightMap.h"

namespace geo {

HeightMap::HeightMap() : root_(0) {
}

HeightMap::HeightMap(const HeightMap& orig) {
    if (orig.root_) {
        root_ = new HeightMapNode(*orig.root_);
    } else {
        root_ = 0;
    }
    mesh_ = orig.mesh_;
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

    // * * * * * * calculate mesh for rasterization * * * * * * * *

    std::vector<std::vector<bool> > visited_grid(grid.size());
    for(unsigned int mx = 0; mx < mx_max; ++mx) {
        visited_grid[mx].resize(my_max, false);
    }

    // top triangles
    for(unsigned int mx = 0; mx < mx_max; ++mx) {
        for(unsigned int my = 0; my < my_max; ++my) {
            double h = grid[mx][my];

            if (!visited_grid[mx][my] && h > 0) {
                unsigned int max_x = mx_max;
                unsigned int max_y = my;
                for(unsigned int y2 = my; y2 < my_max && std::abs(grid[mx][y2] - h) < 1e-10; ++y2) {
                    for(unsigned int x2 = mx; x2 < max_x; ++x2) {
                        if (std::abs(grid[x2][y2] - h) > 1e-10) {
                            max_x = x2;
                            break;
                        }
                    }
                    ++max_y;
                }

                for(unsigned int y2 = my; y2 < max_y; ++y2) {
                    for(unsigned int x2 = mx; x2 < max_x; ++x2) {
                        visited_grid[x2][y2] = true;
                    }
                }

                int p0 = hmap.mesh_.addPoint(resolution * mx, resolution * my, h);
                int p1 = hmap.mesh_.addPoint(resolution * max_x, resolution * my, h);
                int p2 = hmap.mesh_.addPoint(resolution * mx, resolution * max_y, h);
                int p3 = hmap.mesh_.addPoint(resolution * max_x, resolution * max_y, h);

                hmap.mesh_.addTriangle(p0, p1, p2);
                hmap.mesh_.addTriangle(p1, p3, p2);
            }
        }
    }

    // side triangles x-axis
    for(unsigned int mx = 0; mx <= mx_max; ++mx) {

        double h_last = 0;
        double h2_last = 0;
        int my_start = -1;
        for(unsigned int my = 0; my <= my_max; ++my) {
            double h = 0;
            if (mx < mx_max && my < my_max) {
                h = grid[mx][my];
            }

            double h2 = 0;
            if (mx > 0 && my < my_max) {
                h2 = grid[mx - 1][my];
            }

            if (my_start >= 0) {
                if (std::abs(h - h_last) + std::abs(h2 - h2_last) > 1e-10) {
                    // create triangles
                    int p0 = hmap.mesh_.addPoint(resolution * mx, resolution * my_start, h_last);
                    int p1 = hmap.mesh_.addPoint(resolution * mx, resolution * my, h_last);
                    int p2 = hmap.mesh_.addPoint(resolution * mx, resolution * my_start, h2_last);
                    int p3 = hmap.mesh_.addPoint(resolution * mx, resolution * my, h2_last);

                    hmap.mesh_.addTriangle(p1, p2, p0);
                    hmap.mesh_.addTriangle(p2, p1, p3);

                    if (std::abs(h - h2) > 1e-10) {
                        my_start = my;
                    } else {
                        my_start = -1;
                    }
                }
            } else if (std::abs(h - h2) > 1e-10) {
                my_start = my;
            }

            h_last = h;
            h2_last = h2;
        }
    }

    // side triangles y-axis
    for(unsigned int my = 0; my <= my_max; ++my) {
        double h_last = 0;
        double h2_last = 0;
        int mx_start = -1;
        for(unsigned int mx = 0; mx <= mx_max; ++mx) {
            double h = 0;
            if (mx < mx_max && my < my_max) {
                h = grid[mx][my];
            }

            double h2 = 0;
            if (my > 0 && mx < mx_max) {
                h2 = grid[mx][my - 1];
            }

            if (mx_start >= 0) {
                if (std::abs(h - h_last) + std::abs(h2 - h2_last) > 1e-10) {
                    // create triangles
                    int p0 = hmap.mesh_.addPoint(resolution * mx_start, resolution * my, h_last);
                    int p1 = hmap.mesh_.addPoint(resolution * mx, resolution * my, h_last);
                    int p2 = hmap.mesh_.addPoint(resolution * mx_start, resolution * my, h2_last);
                    int p3 = hmap.mesh_.addPoint(resolution * mx, resolution * my, h2_last);

                    hmap.mesh_.addTriangle(p2, p1, p0);
                    hmap.mesh_.addTriangle(p1, p2, p3);

                    if (std::abs(h - h2) > 1e-10) {
                        mx_start = mx;
                    } else {
                        mx_start = -1;
                    }
                }
            } else if (std::abs(h - h2) > 1e-10) {
                mx_start = mx;
            }

            h_last = h;
            h2_last = h2;
        }
    }


    hmap.mesh_.filterOverlappingVertices();

    return hmap;
}

HeightMapNode* HeightMap::createQuadTree(const std::vector<std::vector<double> >& map,
                            unsigned int mx_min, unsigned int my_min,
                            unsigned int mx_max, unsigned int my_max, double resolution) {

    double max_height = 0;
    for(unsigned int mx = mx_min; mx < mx_max; ++mx) {
        for(unsigned int my = my_min; my < my_max; ++my) {
            max_height = std::max(max_height, map[mx][my]);
        }
    }

    if (max_height == 0) {
        return 0;
    }

    Vector3 min_map((double)mx_min * resolution,
                    (double)my_min * resolution, 0);
    Vector3 max_map((double)mx_max * resolution,
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

    }

    return n;
}



}


