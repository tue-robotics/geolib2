#include "geolib/HeightMapNode.h"

namespace geo {

HeightMapNode::HeightMapNode(const Box& box) : box_(box), occupied_(false) {
    for(unsigned int i = 0; i < 4; ++i) {
        children_[i] = 0;
    }
}

HeightMapNode::HeightMapNode(const HeightMapNode& orig) : box_(orig.box_), occupied_(orig.occupied_) {
    for(unsigned int i = 0; i < 4; ++i) {
        if (orig.children_[i]) {
            children_[i] = orig.children_[i]->clone();
        } else {
            children_[i] = 0;
        }
    }
}

HeightMapNode::~HeightMapNode() {

}

HeightMapNode* HeightMapNode::clone() const {
    return new HeightMapNode(*this);
}

bool HeightMapNode::intersect(const Ray& r, float t0, float t1, double& distance) const {
    /*if (box_.intersect(r.origin)) {
        std::cout << "TEST!" << std::endl;
        if (box_.intersect(r, t0, t1, distance)) {
            std::cout << "    YES: " << distance << std::endl;
        }
    }*/

    if (!box_.intersect(r, t0, t1, distance)) {
        return false;
    }

    if (occupied_) {
        return true;
    }

    unsigned int i_child_origin = 5;
    for(unsigned int i = 0; i < 4; ++i) {
        if (children_[i] && children_[i]->box_.intersect(r.origin_)) {
            if (children_[i]->intersect(r, t0, t1, distance)) {
                return true;
            }
            i_child_origin = i;
        }
    }


    for(unsigned int i = 0; i < 4; ++i) {
        if (i != i_child_origin && children_[i] && children_[i]->intersect(r, t0, t1, distance)) {
            return true;
        }
    }

    return false;
}

}
