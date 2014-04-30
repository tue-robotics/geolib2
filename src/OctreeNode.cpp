#include "geolib/OctreeNode.h"
#include "geolib/Octree.h"
#include "geolib/Box.h"

namespace geo {

OctreeNode::OctreeNode(double size, Octree* tree)
    : size_(size), split_(size_ / 2), occupied_(false), tree_(tree) {
    for(unsigned int i = 0; i < 8; ++i) {
        children_[i] = 0;
    }
}

OctreeNode::OctreeNode(const OctreeNode& orig, Octree* tree)
    : size_(orig.size_), split_(orig.split_), occupied_(orig.occupied_), tree_(tree) {
    for(unsigned int i = 0; i < 8; ++i) {
        if (orig.children_[i]) {
            children_[i] = new OctreeNode(*orig.children_[i], tree);
        } else {
            children_[i] = 0;
        }
    }
}

OctreeNode::~OctreeNode() {
    for(unsigned int i = 0; i < 8; ++i) {
        delete children_[i];
    }
}

void OctreeNode::add(const Vector3& p) {
    //std::cout << p << ", size = " << size_ << std::endl;

    if (size_ - 1e-10 < tree_->resolution_) {
        occupied_ = true;
        return;
    }

    int index = 0;

    double x = p.x;
    double y = p.y;
    double z = p.z;

    if (x > split_) { index += 4; x-= split_; }
    if (y > split_) { index += 2; y-= split_; }
    if (z > split_) { index += 1; z-= split_; }

    if (!children_[index]) {
        children_[index] = new OctreeNode(size_ / 2, tree_);
    }
    children_[index]->add(Vector3(x, y, z));
}

void OctreeNode::getCubes(std::vector<Box>& cubes, const Vector3& offset) const {
    if (occupied_) {
        cubes.push_back(Box(offset, offset + Vector3(size_, size_, size_)));
    } else {
        int i = 0;
        for(int x = 0; x <= 1; ++x) {
            for(int y = 0; y <= 1; ++y) {
                for(int z = 0; z <= 1; ++z) {
                    if (children_[i]) {
                        children_[i]->getCubes(cubes, offset + Vector3(split_ * x, split_ * y, split_ * z));
                    }
                    ++i;
                }
            }
        }
    }
}


bool OctreeNode::intersect(const Ray& r, float t0, float t1, double& distance, const Vector3& offset) const {

    //std::cout << r.origin_ << ", t0 = " << t0 << ", t1 = " << t1 << " distance = " << distance << std::endl;
    //std::cout << offset << " - " << offset + Vector3(size_, size_, size_) << std::endl;

    if (occupied_) {
        return true;
    }

    Vector3 o = r.origin_ + t0 * r.direction_;
    if (o.x < 0 || o.y < 0 || o.z < 0
            || o.x > size_ || o.y > size_ || o.z > size_) {
        return false;
    }

    //std::cout << "    o = " << o << std::endl;

    int index = 0;

    double dx = 0;
    double dy = 0;
    double dz = 0;

    if (o.x >= split_) { index += 4; dx = split_; }
    if (o.y >= split_) { index += 2; dy = split_; }
    if (o.z >= split_) { index += 1; dz = split_; }

    if (children_[index]) {
        if (children_[index]->intersect(Ray(r.origin_ - Vector3(dx, dy, dz), r.direction_), t0, t1, distance, offset + Vector3(dx, dy, dz))) {
            return true;
        }

        // is distance already correctly set in this case? If so, no need to calculate it again below
    }

    double dist;
    Box b(Vector3(dx, dy, dz), Vector3(dx + size_ / 2, dy + size_ / 2, dz + size_ / 2));
    b.intersect(Ray(o, -r.direction_), 0, t1 - t0, dist);
    distance = t0 - dist;
    return this->intersect(r, distance + tree_->resolution_ * 0.1, t1, distance, offset);
}

void OctreeNode::raytrace(const Vector3& o, const Vector3& dir, float t0, float t1, const Vector3& offset) {
    if (t1 < t0) {
        return;
    }

    Vector3 newo = o + t0 * dir;
    if (newo.x < 0 || newo.y < 0 || newo.z < 0
            || newo.x > size_ || newo.y > size_ || newo.z > size_) {
        return;
    }

    occupied_ = false;

    int index = 0;

    double dx = 0;
    double dy = 0;
    double dz = 0;

    if (newo.x >= split_) { index += 4; dx = split_; }
    if (newo.y >= split_) { index += 2; dy = split_; }
    if (newo.z >= split_) { index += 1; dz = split_; }

    if (children_[index]) {
        children_[index]->raytrace(o - Vector3(dx, dy, dz), dir, t0, t1, offset + Vector3(dx, dy, dz));
    }

    double dist;
    Box b(Vector3(dx, dy, dz), Vector3(dx + size_ / 2, dy + size_ / 2, dz + size_ / 2));
    b.intersect(Ray(newo, -dir), 0, t1 - t0, dist);
    double distance = t0 - dist;
    this->raytrace(o, dir, distance + tree_->resolution_ * 0.1, t1, offset);
}

bool OctreeNode::intersect(const Vector3& p) const {
    if (occupied_) {
        return true;
    }

    int index = 0;

    double x = p.x;
    double y = p.y;
    double z = p.z;

    if (x > split_) { index += 4; x-= split_; }
    if (y > split_) { index += 2; y-= split_; }
    if (z > split_) { index += 1; z-= split_; }

    if (!children_[index]) {
        return false;
    } else {
        return children_[index]->intersect(p);
    }
}

bool OctreeNode::intersect(const Box& b) const {
    if (occupied_) {
        return true;
    }

    int sx = b.bounds[0].x > split_;
    int ex = b.bounds[1].x > split_;

    int sy = b.bounds[0].y > split_;
    int ey = b.bounds[1].y > split_;

    int sz = b.bounds[0].z > split_;
    int ez = b.bounds[1].z > split_;

    for(int x = sx; x <= ex; ++x) {
        for(int y = sy; y <= ey; ++y) {
            for(int z = sz; z <= ez; ++z) {
                int i = 4 * x + 2 * y + z;
                if (children_[i]) {
                    Vector3 offset(split_ * x, split_ * y, split_ * z);
                    if (children_[i]->intersect(Box(b.bounds[0] - offset, b.bounds[1] - offset))) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

}
