#include <algorithm>

#include <cmath>

#include "geolib/Box.h"
#include "geolib/Mesh.h"
#include "geolib/Octree.h"
#include "geolib/OctreeNode.h"
#include "geolib/Ray.h"
#include "geolib/Shape.h"
#include <vector>

namespace geo
{

Octree::Octree(double size, double resolution) :
    offset_(-size / 2, -size / 2, -size / 2), max_(size / 2, size / 2, size / 2), size_(size),
    root_(new OctreeNode(size, this))
{
    setResolution(resolution);
}

Octree::Octree(const Octree& orig) :
    Shape(orig), resolution_(orig.resolution_), offset_(orig.offset_), max_(orig.max_), size_(orig.size_),
    root_(new OctreeNode(*orig.root_, this))
{
}

Octree::~Octree()
{
    delete root_;
}

Octree* Octree::clone() const
{
    return new Octree(*this);
}

void Octree::clear()
{
    delete root_;
    root_ = new OctreeNode(size_, this);
    invalidateCache();
}

void Octree::add(const Vector3& p)
{
    root_->add(p - offset_);
    invalidateCache();
}

void Octree::getCubes(std::vector<Box>& cubes) const
{
    root_->getCubes(cubes, offset_);
}

double Octree::setResolution(double resolution)
{
    resolution_ = size_;
    while (resolution_ > resolution)
    {
        resolution_ /= 2;
    }
    return resolution_;
}

double Octree::getResolution() const
{
    return resolution_;
}

bool Octree::intersect(const Ray& r, float t0, float t1, double& distance) const
{
    double dist = NAN;
    if (!Box(offset_, max_).intersect(r, t0, t1, dist))
    {
        return false;
    }

    dist = std::max<double>(dist, 0);

    return root_->intersect(Ray(r.getOrigin() - offset_, r.getDirection()),
                            static_cast<float>(t0 + dist + (resolution_ * 0.1)),
                            t1,
                            distance,
                            offset_);
}

double Octree::getMaxRadius() const
{
    return size_;
}

void Octree::raytrace(const Ray& r, float t0, float t1)
{
    double dist = NAN;
    if (!Box(offset_, max_).intersect(r, t0, t1, dist))
    {
        return;
    }

    dist = std::max<double>(dist, 0);

    root_->raytrace(
        r.getOrigin() - offset_, r.getDirection(), static_cast<float>(t0 + dist + (resolution_ * 0.1)), t1, offset_);
    this->add(r.getOrigin() + r.getDirection() * t1);

    invalidateCache();
}

bool Octree::contains(const Vector3& p) const
{
    if (p.x < offset_.x || p.y < offset_.y || p.z < offset_.z || p.x > max_.x || p.y > max_.y || p.z > max_.z)
    {
        return false;
    }

    return root_->contains(p - offset_);
}

bool Octree::intersect(const Box& b) const
{
    const Vector3& min = b.getMin();
    const Vector3& max = b.getMax();
    if (max.x < offset_.x || max.y < offset_.y || max.z < offset_.z || min.x > max_.x || min.y > max_.y ||
        min.z > max_.z)
    {
        return false;
    }

    return root_->intersect(
        Box(Vector3(std::max(min.x, offset_.x), std::max(min.y, offset_.y), std::max(min.z, offset_.z)) - offset_,
            Vector3(std::min(max.x, max_.x), std::min(max.y, max_.y), std::min(max.z, max_.z)) - offset_));
}

bool Octree::intersect(const Vector3& p, const double RADIUS) const
{
    if (RADIUS <= 0)
    {
        return contains(p);
    }

    // The sphere should intersect the cube of the root node
    double d2 = 0;
    for (unsigned int i = 0; i < 3; ++i)
    {
        double const d = std::max({offset_[i] - p[i], 0.0, p[i] - max_[i]});
        d2 += d * d;
    }
    if (d2 > RADIUS * RADIUS)
    {
        return false;
    }

    return root_->intersect(p - offset_, RADIUS);
}

Box Octree::getBoundingBox() const
{
    if (!bounding_box_cache_valid_)
    {
        // Same result as for an empty mesh, in case nothing is occupied
        bounding_box_min_cache_ = Vector3(1e9, 1e9, 1e9);
        bounding_box_max_cache_ = Vector3(-1e9, -1e9, -1e9);
        root_->extendBoundingBox(bounding_box_min_cache_, bounding_box_max_cache_, offset_);
        bounding_box_cache_valid_ = true;
    }
    return {bounding_box_min_cache_, bounding_box_max_cache_};
}

const Mesh& Octree::getMesh() const
{
    if (mesh_.empty())
    {
        std::vector<Box> cubes;
        getCubes(cubes);

        for (auto& b : cubes)
        {
            mesh_.add(b.getMesh());
        }

        mesh_.filterOverlappingVertices();
    }

    return mesh_;
}

} // namespace geo
