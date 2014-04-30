#include "geolib/Mesh.h"

namespace geo {

Mesh::Mesh() : max_radius_(0) {
}

Mesh::~Mesh() {
}

int Mesh::addPoint(double x, double y, double z) {
    int i = points_.size();
    points_.push_back(Vector3(x, y, z));
    triangles_.clear();
    return i;
}

int Mesh::addPoint(const geo::Vector3& p) {
    int i = points_.size();
    points_.push_back(p);
    triangles_.clear();
    return i;
}

void Mesh::addTriangle(int i1, int i2, int i3) {
    triangles_i_.push_back(TriangleI(i1, i2, i3));
    triangles_.clear();
}

void Mesh::add(const Mesh& mesh) {
    int i_start = points_.size();
    points_.insert(points_.end(), mesh.points_.begin(), mesh.points_.end());

    for(std::vector<TriangleI>::const_iterator it = mesh.triangles_i_.begin(); it != mesh.triangles_i_.end(); ++it) {
        triangles_i_.push_back(TriangleI(it->i1_ + i_start, it->i2_ + i_start, it->i3_ + i_start));
    }

    triangles_.clear();
}

//void Mesh::removeTriangles(const std::set<int>& indices) {
//    for(std::set<int>::const_reverse_iterator it = indices.rbegin(); it != indices.rend(); ++it) {
//        int i_triangle = *it;
//        if (i_triangle <= (int)triangles_i_.size() - 1) {
//            triangles_i_[i_triangle] = triangles_i_.back();
//            triangles_i_.pop_back();
//        }
//    }

//    triangles_.clear();
//}

void Mesh::copyAndRemoveTriangles(const std::set<int>& indices, geo::Mesh& target) const {
    target.clear();

    std::vector<int> vertex_mapping(points_.size(), -1);

    std::set<int>::const_iterator it = indices.begin();
    for(int i = 0; i < (int)triangles_i_.size() && it != indices.end(); ++i) {
        int i_triangle = *it;
        if (i == i_triangle) {
            ++it;
        } else {
            int i1 = foo(triangles_i_[i].i1_, points_, vertex_mapping, target.points_);
            int i2 = foo(triangles_i_[i].i2_, points_, vertex_mapping, target.points_);
            int i3 = foo(triangles_i_[i].i3_, points_, vertex_mapping, target.points_);
            target.triangles_i_.push_back(TriangleI(i1, i2, i3));
        }
    }
}

const std::vector<Vector3>& Mesh::getPoints() const {
    return points_;
}

const std::vector<TriangleI>& Mesh::getTriangleIs() const {
    return triangles_i_;
}

const std::vector<Triangle>& Mesh::getTriangles() const {
    if (triangles_.empty()) {
        for(std::vector<TriangleI>::const_iterator it = triangles_i_.begin(); it != triangles_i_.end(); ++it) {
            triangles_.push_back(Triangle(points_[it->i1_], points_[it->i2_], points_[it->i3_]));
        }
    }
    return triangles_;
}

Mesh Mesh::getTransformed(const geo::Transform t) const {
    Mesh m;
    m.triangles_i_ = this->triangles_i_;
    m.points_.resize(this->points_.size());

    for(unsigned int i = 0; i < points_.size(); ++i) {
        m.points_[i] = t * points_[i];
    }
    return m;
}

void Mesh::filterOverlappingVertices() {
    std::vector<Vector3> old_points = points_;
    points_.clear();

    std::map<int, std::map<int, std::map<int, int> > > xyz_map;
    std::vector<int> i_map(old_points.size());

    for(unsigned int j = 0; j < old_points.size(); ++j) {
        const Vector3& v = old_points[j];

        int ix = 1000 * v.x;
        int iy = 1000 * v.y;
        int iz = 1000 * v.z;

        bool match = false;
        std::map<int, std::map<int, std::map<int, int> > >::iterator it1 = xyz_map.find(ix);
        if (it1 != xyz_map.end()) {
            std::map<int, std::map<int, int> >::iterator it2 = it1->second.find(iy);
            if (it2 != it1->second.end()) {
                std::map<int, int>::iterator it3 = it2->second.find(iz);
                if (it3 != it2->second.end()) {
                    i_map[j] = it3->second;
                    match = true;
                }
            }
        }

        if (!match) {
            int ip = this->addPoint(v.x, v.y, v.z);
            xyz_map[ix][iy][iz] = ip;
            i_map[j] = ip;
        }
    }

    for(std::vector<TriangleI>::iterator it = triangles_i_.begin(); it != triangles_i_.end(); ++it) {
        it->i1_ = i_map[it->i1_];
        it->i2_ = i_map[it->i2_];
        it->i3_ = i_map[it->i3_];
    }
}

double Mesh::getMaxRadius() const {
    if (max_radius_ == 0) {
        double max_radius_sq = 0;
        for(std::vector<Vector3>::const_iterator it = points_.begin(); it != points_.end(); ++it) {
            max_radius_sq = std::max(max_radius_sq, (double)it->length2());
        }
        max_radius_ = sqrt(max_radius_sq);
    }
    return max_radius_;
}

}
