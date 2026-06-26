#include <utility>

#include "geolib/Box.h"
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

HeightMapNode::~HeightMapNode() = default;

// NOLINTNEXTLINE(misc-no-recursion)
HeightMapNode* HeightMapNode::clone() const
{
    return new HeightMapNode(*this);
}

// NOLINTNEXTLINE(misc-no-recursion)
bool HeightMapNode::intersect(const Ray& r, float t0, float t1, double& distance) const
{
    /*if (box_.intersect(r.origin)) {
        std::cout << "TEST!" << std::endl;
        if (box_.intersect(r, t0, t1, distance)) {
            std::cout << "    YES: " << distance << std::endl;
        }
    }*/

    if (!box_.intersect(r, t0, t1, distance))
    {
        return false;
    }

    if (occupied_)
    {
        return true;
    }

    unsigned int i_child_origin = 5;
    for (unsigned int i = 0; i < 4; ++i)
    {
        if (children_[i] && children_[i]->box_.contains(r.getOrigin()))
        {
            if (children_[i]->intersect(r, t0, t1, distance))
            {
                return true;
            }
            i_child_origin = i;
        }
    }

    for (unsigned int i = 0; i < 4; ++i)
    {
        if (i != i_child_origin && children_[i] && children_[i]->intersect(r, t0, t1, distance))
        {
            return true;
        }
    }

    return false;
}

} // namespace geo
