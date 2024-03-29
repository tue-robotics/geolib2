#include "geolib/Mesh.h"
#include "geolib/math_types.h"

#include <map>

namespace geo {

Mesh::Mesh() {
    invalidateCache();
}

Mesh::~Mesh() {
}

unsigned int Mesh::addPoint(double x, double y, double z) {
    return addPoint(Vector3(x, y, z));
}

unsigned int Mesh::addPoint(const geo::Vector3& p) {
    std::size_t i = points_.size();
    points_.push_back(p);
    invalidateCache();
    return i;
}

void Mesh::addTriangle(unsigned int i1, unsigned int i2, unsigned int i3) {
    triangles_i_.push_back(TriangleI(i1, i2, i3));
    invalidateCache();
}

void Mesh::add(const Mesh& mesh) {
    unsigned int i_start = points_.size();
    points_.insert(points_.end(), mesh.points_.begin(), mesh.points_.end());

    for(std::vector<TriangleI>::const_iterator it = mesh.triangles_i_.begin(); it != mesh.triangles_i_.end(); ++it) {
        triangles_i_.push_back(TriangleI(it->i1_ + i_start, it->i2_ + i_start, it->i3_ + i_start));
    }

    invalidateCache();
}

const std::vector<Vector3>& Mesh::getPoints() const {
    return points_;
}

const std::vector<TriangleI>& Mesh::getTriangleIs() const {
    return triangles_i_;
}

const std::vector<Triangle>& Mesh::getTriangles() const {
    if (triangles_cache_.empty()) {
        for(std::vector<TriangleI>::const_iterator it = triangles_i_.begin(); it != triangles_i_.end(); ++it) {
            triangles_cache_.push_back(Triangle(points_[it->i1_], points_[it->i2_], points_[it->i3_]));
        }
    }
    return triangles_cache_;
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

/**
 * Custom compare "<" class vor 3D integer vectors.
 * http://stackoverflow.com/questions/5733254/create-an-own-comparator-for-map
 * http://en.cppreference.com/w/cpp/concept/Compare
 */
struct CompareVec3i {
    /** Comparator function. */
    bool operator() (const Vec3i &a, const Vec3i &b) const {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    }
};

void Mesh::filterOverlappingVertices() {
    std::vector<Vector3> old_points = points_;
    points_.clear();

    std::map<Vec3i, int, CompareVec3i> xyz_map;
    std::vector<int> i_map(old_points.size());

    for(unsigned int j = 0; j < old_points.size(); ++j) {
        const Vector3 &v = old_points[j];

        int ix = 1000 * v.x;
        int iy = 1000 * v.y;
        int iz = 1000 * v.z;

        Vec3i v3int(ix, iy, iz);
        std::map<Vec3i, int>::iterator iter = xyz_map.find(v3int);
        if (iter != xyz_map.end()) {
            i_map[j] = iter->second;
        } else {
            int ip = this->addPoint(v);
            xyz_map[v3int] = ip;
            i_map[j] = ip;
        }
    }

    for(std::vector<TriangleI>::iterator it = triangles_i_.begin(); it != triangles_i_.end(); ++it) {
        it->i1_ = i_map[it->i1_];
        it->i2_ = i_map[it->i2_];
        it->i3_ = i_map[it->i3_];
    }
}

double Mesh::getSquaredMaxRadius() const {
    if (max_radius_squared_cache_ == 0) {
        for(std::vector<Vector3>::const_iterator it = points_.begin(); it != points_.end(); ++it) {
            max_radius_squared_cache_ = std::max(max_radius_squared_cache_, (double)it->length2());
        }
    }
    return max_radius_squared_cache_;
}

double Mesh::getMaxRadius() const {
    if (max_radius_cache_ == 0) {
        max_radius_cache_ = sqrt(getSquaredMaxRadius());
    }
    return max_radius_cache_;
}

const geo::Vector3 Mesh::getTriangleNormal(unsigned int index) const {
    const geo::TriangleI& t = getTriangleIs()[index];

    const geo::Vector3& p1 = points_[t.i1_];
    const geo::Vector3& p2 = points_[t.i2_];
    const geo::Vector3& p3 = points_[t.i3_];

    return ((p3 - p1).cross(p2 - p1)).normalized();
}

}
