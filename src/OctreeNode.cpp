#include <cmath>

#include "geolib/Box.h"
#include "geolib/datatypes.h"
#include "geolib/Octree.h"
#include "geolib/OctreeNode.h"
#include "geolib/Ray.h"
#include <vector>

namespace geo
{

OctreeNode::OctreeNode(double size, Octree* tree) : size_(size), split_(size_ / 2), occupied_(false), tree_(tree)
{
    for (auto& i : children_)
    {
        i = nullptr;
    }
}

// NOLINTNEXTLINE(misc-no-recursion)
OctreeNode::OctreeNode(const OctreeNode& orig, Octree* tree) :
    size_(orig.size_), split_(orig.split_), occupied_(orig.occupied_), tree_(tree)
{
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (orig.children_[i])
        {
            children_[i] = new OctreeNode(*orig.children_[i], tree);
        }
        else
        {
            children_[i] = nullptr;
        }
    }
}

OctreeNode::~OctreeNode()
{
    for (auto& i : children_)
    {
        delete i;
    }
}

// NOLINTNEXTLINE(misc-no-recursion)
void OctreeNode::add(const Vector3& p)
{
    // std::cout << p << ", size = " << size_ << std::endl;

    if (size_ - 1e-10 < tree_->resolution_)
    {
        occupied_ = true;
        return;
    }

    int index = 0;

    double x = p.x;
    double y = p.y;
    double z = p.z;

    if (x > split_)
    {
        index += 4;
        x -= split_;
    }
    if (y > split_)
    {
        index += 2;
        y -= split_;
    }
    if (z > split_)
    {
        index += 1;
        z -= split_;
    }

    if (!children_[index])
    {
        children_[index] = new OctreeNode(size_ / 2, tree_);
    }
    children_[index]->add(Vector3(x, y, z));
}

// NOLINTNEXTLINE(misc-no-recursion)
void OctreeNode::getCubes(std::vector<Box>& cubes, const Vector3& offset) const
{
    if (occupied_)
    {
        cubes.emplace_back(offset, offset + Vector3(size_, size_, size_));
    }
    else
    {
        int i = 0;
        for (int x = 0; x <= 1; ++x)
        {
            for (int y = 0; y <= 1; ++y)
            {
                for (int z = 0; z <= 1; ++z)
                {
                    if (children_[i])
                    {
                        children_[i]->getCubes(cubes, offset + Vector3(split_ * x, split_ * y, split_ * z));
                    }
                    ++i;
                }
            }
        }
    }
}

// NOLINTNEXTLINE(misc-no-recursion)
bool OctreeNode::intersect(const Ray& r, float t0, float t1, double& distance, const Vector3& offset) const
{

    // std::cout << r.getOrigin() << ", t0 = " << t0 << ", t1 = " << t1 << " distance = " << distance << std::endl;
    // std::cout << offset << " - " << offset + Vector3(size_, size_, size_) << std::endl;

    if (occupied_)
    {
        return true;
    }

    Vector3 const o = r.getOrigin() + t0 * r.getDirection();
    if (o.x < 0 || o.y < 0 || o.z < 0 || o.x > size_ || o.y > size_ || o.z > size_)
    {
        return false;
    }

    // std::cout << "    o = " << o << std::endl;

    int index = 0;

    double dx = 0;
    double dy = 0;
    double dz = 0;

    if (o.x >= split_)
    {
        index += 4;
        dx = split_;
    }
    if (o.y >= split_)
    {
        index += 2;
        dy = split_;
    }
    if (o.z >= split_)
    {
        index += 1;
        dz = split_;
    }

    if (children_[index])
    {
        if (children_[index]->intersect(Ray(r.getOrigin() - Vector3(dx, dy, dz), r.getDirection()),
                                        t0,
                                        t1,
                                        distance,
                                        offset + Vector3(dx, dy, dz)))
        {
            return true;
        }

        // is distance already correctly set in this case? If so, no need to calculate it again below
    }

    double dist = NAN;
    Box const b(Vector3(dx, dy, dz), Vector3(dx + (size_ / 2), dy + (size_ / 2), dz + (size_ / 2)));
    b.intersect(Ray(o, -r.getDirection()), 0, t1 - t0, dist);
    distance = t0 - dist;
    return this->intersect(r, static_cast<float>(distance + (tree_->resolution_ * 0.1)), t1, distance, offset);
}

// NOLINTNEXTLINE(misc-no-recursion)
void OctreeNode::raytrace(const Vector3& o, const Vector3& dir, float t0, float t1, const Vector3& offset)
{
    if (t1 < t0)
    {
        return;
    }

    Vector3 const newo = o + t0 * dir;
    if (newo.x < 0 || newo.y < 0 || newo.z < 0 || newo.x > size_ || newo.y > size_ || newo.z > size_)
    {
        return;
    }

    occupied_ = false;

    int index = 0;

    double dx = 0;
    double dy = 0;
    double dz = 0;

    if (newo.x >= split_)
    {
        index += 4;
        dx = split_;
    }
    if (newo.y >= split_)
    {
        index += 2;
        dy = split_;
    }
    if (newo.z >= split_)
    {
        index += 1;
        dz = split_;
    }

    if (children_[index])
    {
        children_[index]->raytrace(o - Vector3(dx, dy, dz), dir, t0, t1, offset + Vector3(dx, dy, dz));
    }

    double dist = NAN;
    Box const b(Vector3(dx, dy, dz), Vector3(dx + (size_ / 2), dy + (size_ / 2), dz + (size_ / 2)));
    b.intersect(Ray(newo, -dir), 0, t1 - t0, dist);
    double const distance = t0 - dist;
    this->raytrace(o, dir, static_cast<float>(distance + (tree_->resolution_ * 0.1)), t1, offset);
}

// NOLINTNEXTLINE(misc-no-recursion)
bool OctreeNode::contains(const Vector3& p) const
{
    if (occupied_)
    {
        return true;
    }

    int index = 0;

    const double x = p.x;
    const double y = p.y;
    const double z = p.z;

    if (x > split_)
    {
        index += 4;
    }
    if (y > split_)
    {
        index += 2;
    }
    if (z > split_)
    {
        index += 1;
    }

    if (!children_[index])
    {
        return false;
    }

    return children_[index]->contains(p);
}

// NOLINTNEXTLINE(misc-no-recursion)
bool OctreeNode::intersect(const Box& b) const
{
    if (occupied_)
    {
        return true;
    }
    const Vector3& min = b.getMin();
    const Vector3& max = b.getMax();

    int const sx = min.x > split_;
    int const ex = max.x > split_;

    int const sy = min.y > split_;
    int const ey = max.y > split_;

    int const sz = min.z > split_;
    int const ez = max.z > split_;

    for (int x = sx; x <= ex; ++x)
    {
        for (int y = sy; y <= ey; ++y)
        {
            for (int z = sz; z <= ez; ++z)
            {
                int const i = (4 * x) + (2 * y) + z;
                if (children_[i])
                {
                    Vector3 const offset(split_ * x, split_ * y, split_ * z);
                    if (children_[i]->intersect(Box(min - offset, max - offset)))
                    {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

} // namespace geo
