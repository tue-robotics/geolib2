#include <utility>

#include "geolib/Box.h"
#include "geolib/datatypes.h"
#include "geolib/HeightMapNode.h"
#include "geolib/Ray.h"

namespace geo
{

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

// NOLINTNEXTLINE(misc-no-recursion)
HeightMapNode& HeightMapNode::operator=(const HeightMapNode& rhs)
{
    if (this != &rhs)
    {
        box_ = rhs.box_;
        occupied_ = rhs.occupied_;
        for (unsigned int i = 0; i < 4; ++i)
        {
            delete children_[i];
            children_[i] = rhs.children_[i] ? rhs.children_[i]->clone() : nullptr;
        }
    }
    return *this;
}

// NOLINTNEXTLINE(misc-no-recursion)
HeightMapNode::~HeightMapNode()
{
    for (const HeightMapNode* child : children_)
    {
        delete child;
    }
}

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
        if (child && child->intersect(r, t0, hit ? static_cast<float>(distance) : t1, d) && (!hit || d < distance))
        {
            distance = d;
            hit = true;
        }
    }

    return hit;
}

// NOLINTNEXTLINE(misc-no-recursion)
bool HeightMapNode::intersect(const Vector3& p, double radius) const
{
    if (!box_.intersect(p, radius))
    {
        return false;
    }

    if (occupied_)
    {
        return true;
    }

    // NOLINTNEXTLINE(readability-use-anyofallof) - std::any_of with a lambda triggers misc-no-recursion
    for (const HeightMapNode* child : children_)
    {
        if (child && child->intersect(p, radius))
        {
            return true;
        }
    }

    return false;
}

// NOLINTNEXTLINE(misc-no-recursion)
bool HeightMapNode::contains(const Vector3& p) const
{
    if (!box_.contains(p))
    {
        return false;
    }

    if (occupied_)
    {
        return true;
    }

    // NOLINTNEXTLINE(readability-use-anyofallof) - std::any_of with a lambda triggers misc-no-recursion
    for (const HeightMapNode* child : children_)
    {
        if (child && child->contains(p))
        {
            return true;
        }
    }

    return false;
}

} // namespace geo
