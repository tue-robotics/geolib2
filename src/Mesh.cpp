#include "geolib/Mesh.h"
#include "geolib/datatypes.h"
#include "geolib/math_types.h"
#include "geolib/Triangle.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <vector>

namespace geo
{

Mesh::Mesh()
{
    invalidateCache();
}

Mesh::~Mesh() = default;

unsigned int Mesh::addPoint(double x, double y, double z)
{
    return addPoint(Vector3(x, y, z));
}

unsigned int Mesh::addPoint(const geo::Vector3& p)
{
    std::size_t const i = points_.size();
    points_.push_back(p);
    invalidateCache();
    return i;
}

void Mesh::addTriangle(unsigned int i1, unsigned int i2, unsigned int i3)
{
    triangles_i_.emplace_back(i1, i2, i3);
    invalidateCache();
}

void Mesh::add(const Mesh& mesh)
{
    unsigned int const i_start = points_.size();
    points_.insert(points_.end(), mesh.points_.begin(), mesh.points_.end());

    for (auto it : mesh.triangles_i_)
    {
        triangles_i_.emplace_back(it.i1_ + i_start, it.i2_ + i_start, it.i3_ + i_start);
    }

    invalidateCache();
}

const std::vector<Vector3>& Mesh::getPoints() const
{
    return points_;
}

const std::vector<TriangleI>& Mesh::getTriangleIs() const
{
    return triangles_i_;
}

const std::vector<Triangle>& Mesh::getTriangles() const
{
    if (triangles_cache_.empty())
    {
        for (auto it : triangles_i_)
        {
            triangles_cache_.emplace_back(points_[it.i1_], points_[it.i2_], points_[it.i3_]);
        }
    }
    return triangles_cache_;
}

Mesh Mesh::getTransformed(const geo::Transform& T) const
{
    Mesh m;
    m.triangles_i_ = this->triangles_i_;
    m.points_.resize(this->points_.size());

    for (unsigned int i = 0; i < points_.size(); ++i)
    {
        m.points_[i] = T * points_[i];
    }
    return m;
}

/**
 * Custom compare "<" class vor 3D integer vectors.
 * http://stackoverflow.com/questions/5733254/create-an-own-comparator-for-map
 * http://en.cppreference.com/w/cpp/concept/Compare
 */
struct CompareVec3i
{
    /** Comparator function. */
    bool operator()(const Vec3i& a, const Vec3i& b) const
    {
        if (a.x != b.x)
            return a.x < b.x;
        if (a.y != b.y)
            return a.y < b.y;
        return a.z < b.z;
    }
};

void Mesh::filterOverlappingVertices()
{
    std::vector<Vector3> old_points = points_;
    points_.clear();

    std::map<Vec3i, int, CompareVec3i> xyz_map;
    std::vector<int> i_map(old_points.size());

    for (unsigned int j = 0; j < old_points.size(); ++j)
    {
        const Vector3& v = old_points[j];

        int const ix = static_cast<int>(1000 * v.x);
        int const iy = static_cast<int>(1000 * v.y);
        int const iz = static_cast<int>(1000 * v.z);

        Vec3i const v3int(ix, iy, iz);
        auto const iter = xyz_map.find(v3int);
        if (iter != xyz_map.end())
        {
            i_map[j] = iter->second;
        }
        else
        {
            int const ip = static_cast<int>(this->addPoint(v));
            xyz_map[v3int] = ip;
            i_map[j] = ip;
        }
    }

    for (auto& it : triangles_i_)
    {
        it.i1_ = i_map[it.i1_];
        it.i2_ = i_map[it.i2_];
        it.i3_ = i_map[it.i3_];
    }
}

double Mesh::getSquaredMaxRadius() const
{
    if (max_radius_squared_cache_ == 0)
    {
        for (const auto& point : points_)
        {
            max_radius_squared_cache_ = std::max(max_radius_squared_cache_, static_cast<double>(point.length2()));
        }
    }
    return max_radius_squared_cache_;
}

double Mesh::getMaxRadius() const
{
    if (max_radius_cache_ == 0)
    {
        max_radius_cache_ = sqrt(getSquaredMaxRadius());
    }
    return max_radius_cache_;
}

geo::Vector3 Mesh::getTriangleNormal(unsigned int index) const
{
    const geo::TriangleI& t = getTriangleIs()[index];

    const geo::Vector3& p1 = points_[t.i1_];
    const geo::Vector3& p2 = points_[t.i2_];
    const geo::Vector3& p3 = points_[t.i3_];

    return ((p3 - p1).cross(p2 - p1)).normalized();
}

} // namespace geo
