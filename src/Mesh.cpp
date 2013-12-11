#include "geolib/Mesh.h"

namespace geo {

Mesh::Mesh() {

}

Mesh::~Mesh() {

}


int Mesh::addPoint(double x, double y, double z) {
    int i = points_.size();
    points_.push_back(tf::Vector3(x, y, z));
    triangles_.clear();
    return i;
}

void Mesh::addTriangle(int i1, int i2, int i3) {
    triangles_i_.push_back(Index3(i1, i2, i3));
    triangles_.clear();
}

void Mesh::add(const Mesh& mesh) {
    int i_start = points_.size();
    points_.insert(points_.end(), mesh.points_.begin(), mesh.points_.end());

    for(std::vector<Index3>::const_iterator it = mesh.triangles_i_.begin(); it != mesh.triangles_i_.end(); ++it) {
        triangles_i_.push_back(Index3(it->i1_ + i_start, it->i2_ + i_start, it->i3_ + i_start));
    }

    triangles_.clear();
}

const std::vector<Triangle>& Mesh::getTriangles() const {
    if (triangles_.empty()) {
        for(std::vector<Index3>::const_iterator it = triangles_i_.begin(); it != triangles_i_.end(); ++it) {
            triangles_.push_back(Triangle(points_[it->i1_], points_[it->i2_], points_[it->i3_]));
        }
    }
    return triangles_;
}

Mesh Mesh::getTransformed(const tf::Transform t) const {
    Mesh m;
    m.triangles_i_ = this->triangles_i_;
    m.points_.resize(this->points_.size());

    for(unsigned int i = 0; i < points_.size(); ++i) {
        m.points_[i] = t * points_[i];
    }
    return m;
}

}
