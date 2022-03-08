#include "geolib/Octree.h"
#include "geolib/Box.h"

namespace geo {

Octree::Octree(double size, double resolution) : offset_(-size / 2, -size / 2, -size / 2),
        max_(size / 2, size / 2, size / 2), size_(size), root_(new OctreeNode(size, this)) {
    setResolution(resolution);
}

Octree::Octree(const Octree& orig) : resolution_(orig.resolution_), offset_(orig.offset_),
    max_(orig.max_), size_(orig.size_), root_(new OctreeNode(*orig.root_, this)) {
}

Octree::~Octree() {
    delete root_;
}

Octree* Octree::clone() const {
    return new Octree(*this);
}

void Octree::clear() {
    delete root_;
    root_ = new OctreeNode(size_, this);
    mesh_.clear();
}

void Octree::add(const Vector3& p) {
    root_->add(p - offset_);
    mesh_.clear();
}

void Octree::getCubes(std::vector<Box>& cubes) const {
    root_->getCubes(cubes, offset_);
}

double Octree::setResolution(double resolution) {
    resolution_ = size_;
    while(resolution_ > resolution) {
        resolution_ /= 2;
    }
    return resolution_;
}

double Octree::getResolution() const {
    return resolution_;
}

bool Octree::intersect(const Ray& r, float t0, float t1, double& distance) const {
    double dist;
    if (!Box(offset_, max_).intersect(r, t0, t1, dist)) {
        return false;
    }

    if (dist < 0) {
        // origin of the ray is within the root cube, so set dist to 0
        dist = 0;
    }

    return root_->intersect(Ray(r.getOrigin() - offset_, r.getDirection()), t0 + dist + resolution_ * 0.1, t1, distance, offset_);
}

double Octree::getMaxRadius() const {
    return size_;
}

void Octree::raytrace(const Ray& r, float t0, float t1) {
    double dist;
    if (!Box(offset_, max_).intersect(r, t0, t1, dist)) {
        return;
    }

    if (dist < 0) {
        // origin of the ray is within the root cube, so set dist to 0
        dist = 0;
    }

    root_->raytrace(r.getOrigin() - offset_, r.getDirection(), t0 + dist + resolution_ * 0.1, t1, offset_);
    this->add(r.getOrigin() + r.getDirection() * t1);

    mesh_.clear();
}

bool Octree::contains(const Vector3& p) const {
    if (p.x < offset_.x || p.y < offset_.y || p.z < offset_.z
            || p.x > max_.x || p.y > max_.y || p.z > max_.z) {
        return false;
    }

    return root_->contains(p - offset_);
}

bool Octree::intersect(const Box& b) const {
    const Vector3& min = b.getMin();
    const Vector3& max = b.getMax();
    if (max.x < offset_.x || max.y < offset_.y || max.z < offset_.z
            || min.x > max_.x || min.y > max_.y || min.z > max_.z) {
        return false;
    }

    return root_->intersect(Box(Vector3(std::max(min.x, offset_.x),
                                        std::max(min.y, offset_.y),
                                        std::max(min.z, offset_.z)) - offset_,
                                Vector3(std::min(max.x, max_.x),
                                        std::min(max.y, max_.y),
                                        std::min(max.z, max_.z)) - offset_));
}

const Mesh& Octree::getMesh() const {
    if (mesh_.empty()) {
        std::vector<Box> cubes;
        getCubes(cubes);

        for(std::vector<Box>::iterator it = cubes.begin(); it != cubes.end(); ++it) {
            Box& b = *it;
            mesh_.add(b.getMesh());
        }

        mesh_.filterOverlappingVertices();
    }

    return mesh_;
}

}
