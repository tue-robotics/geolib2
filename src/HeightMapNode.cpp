#include <cmath>
#include <limits>
#include <utility>

#include "geolib/Box.h"
#include "geolib/HeightMapNode.h"
#include "geolib/Ray.h"

namespace geo
{

namespace
{

/**
 * @brief Smallest float which is not smaller than @b d. Used to pass a distance as (float) maximum distance, without
 * discarding hits up to that distance due to rounding down.
 */
float roundUpToFloat(double d)
{
    auto const f = static_cast<float>(d);
    return f < d ? std::nextafter(f, std::numeric_limits<float>::infinity()) : f;
}

} // namespace

HeightMapNode::HeightMapNode(Box box) : box_(std::move(box)), occupied_(false)
{
    for (auto& i : children_)
    {
        i = nullptr;
    }
}

// NOLINTNEXTLINE(misc-no-recursion)
HeightMapNode::HeightMapNode(const HeightMapNode& orig) : box_(orig.box_), occupied_(orig.occupied_)
{
    for (unsigned int i = 0; i < 4; ++i)
    {
        if (orig.children_[i])
        {
            children_[i] = orig.children_[i]->clone();
        }
        else
        {
            children_[i] = nullptr;
        }
    }
}

HeightMapNode::HeightMapNode(HeightMapNode&& orig) noexcept : box_(std::move(orig.box_)), occupied_(orig.occupied_)
{
    for (unsigned int i = 0; i < 4; ++i)
    {
        if (orig.children_[i])
        {
            children_[i] = orig.children_[i];
            orig.children_[i] = nullptr;
        }
        else
        {
            children_[i] = nullptr;
        }
    }
}

HeightMapNode::~HeightMapNode() = default;

// NOLINTNEXTLINE(misc-no-recursion)
HeightMapNode* HeightMapNode::clone() const
{
    return new HeightMapNode(*this);
}

// NOLINTNEXTLINE(misc-no-recursion)
bool HeightMapNode::intersect(const Ray& r, float t0, float t1, double& distance) const
{
    if (!box_.intersect(r, t0, t1, distance))
    {
        return false;
    }

    if (occupied_)
    {
        return true;
    }

    // Find the nearest hit of all children. Children which are further away than the nearest hit so far, are culled by
    // their bounding box.
    bool hit = false;
    for (const HeightMapNode* child : children_)
    {
        double d = 0;
        if (child && child->intersect(r, t0, hit ? roundUpToFloat(distance) : t1, d) && (!hit || d < distance))
        {
            distance = d;
            hit = true;
        }
    }

    return hit;
}

} // namespace geo
