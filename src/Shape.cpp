#include "geolib/Shape.h"
#include "geolib/Box.h"
#include "geolib/datatypes.h"
#include "geolib/Mesh.h"
#include "geolib/MeshBVH.h"
#include "geolib/Ray.h"

#include <algorithm>
#include <cmath>
#include <ios>
#include <istream>
#include <ostream>
#include <string>
#include <vector>

namespace geo
{

const std::string Shape::TYPE = "mesh";

Shape::Shape() : bounding_box_cache_valid_(false) {}

Shape::~Shape() = default;

Shape* Shape::clone() const
{
    return new Shape(*this);
}

bool Shape::intersect(const Ray& r, float t0, float t1, double& distance) const
{
    const Mesh& mesh = getMesh();
    const Vector3& o = r.getOrigin();
    const Vector3& dir = r.getDirection();

    // Ray should pass the bounding sphere of the mesh. The squared distance of the line to the origin is calculated
    // with the cross product, as |o|^2 - (o dot dir)^2 suffers from cancellation for distant origins. A margin is used,
    // such that rays touching the sphere are not rejected due to roundoff.
    if (o.cross(dir).length2() > mesh.getSquaredMaxRadius() * dir.length2() * (1 + 1e-9))
        return false;

    return mesh.getBVH()->intersect(r, t0, t1, distance);
}

/**
 * The sphere intersects the shape if the distance from its center to the surface is at most its radius, or if its
 * center lies within the shape.
 **/
bool Shape::intersect(const Vector3& p, const double RADIUS) const
{
    const Mesh& mesh = getMesh();
    if (p.length() - RADIUS > mesh.getMaxRadius())
    {
        return false;
    }

    if (RADIUS > 0. && mesh.getBVH()->intersectSurface(p, RADIUS))
    {
        return true;
    }

    return contains(p);
}

/**
 * See MeshBVH::contains: a point is inside if the absolute generalized winding number of the mesh around it is larger
 * than 0.5. Points on the surface are considered to be inside.
 **/
bool Shape::contains(const Vector3& p) const
{
    const Mesh& mesh = getMesh();
    if (p.length2() > mesh.getSquaredMaxRadius() * (1 + 1e-9))
    {
        return false;
    }

    return mesh.getBVH()->contains(p);
}

Box Shape::getBoundingBox() const
{
    if (!bounding_box_cache_valid_)
    {
        const Mesh& mesh = getMesh();
        const std::vector<geo::Vector3>& points = mesh.getPoints();
        double x_min = 1e9;
        double y_min = 1e9;
        double z_min = 1e9;
        double x_max = -1e9;
        double y_max = -1e9;
        double z_max = -1e9;
        for (const auto& point : points)
        {
            x_min = std::min<double>(point.x, x_min);
            x_max = std::max<double>(point.x, x_max);
            y_min = std::min<double>(point.y, y_min);
            y_max = std::max<double>(point.y, y_max);
            z_min = std::min<double>(point.z, z_min);
            z_max = std::max<double>(point.z, z_max);
        }
        bounding_box_min_cache_ = geo::Vector3(x_min, y_min, z_min);
        bounding_box_max_cache_ = geo::Vector3(x_max, y_max, z_max);
        bounding_box_cache_valid_ = true;
    }
    return {bounding_box_min_cache_, bounding_box_max_cache_};
}

const Mesh& Shape::getMesh() const
{
    return mesh_;
}

void Shape::setMesh(const Mesh& mesh)
{
    bounding_box_cache_valid_ = false;
    mesh_ = mesh;
}

double Shape::getMaxRadius() const
{
    const Mesh& mesh = getMesh();
    return mesh.getMaxRadius();
}

bool Shape::write(std::ostream& output) const
{
    std::string const type = "mesh    ";
    output.write(type.c_str(), static_cast<std::streamsize>(type.size()));

    const Mesh& mesh = getMesh();
    const std::vector<geo::Vector3>& points = mesh.getPoints();
    int p_size = static_cast<int>(points.size());
    output.write(reinterpret_cast<char*>(&p_size), sizeof(p_size));
    for (const auto& v : points)
    {
        double x = v.x;
        double y = v.y;
        double z = v.z;

        output.write(reinterpret_cast<char*>(&x), sizeof(x));
        output.write(reinterpret_cast<char*>(&y), sizeof(y));
        output.write(reinterpret_cast<char*>(&z), sizeof(z));
    }

    const std::vector<geo::TriangleI> triangles = mesh.getTriangleIs();
    int t_size = static_cast<int>(triangles.size());
    output.write(reinterpret_cast<char*>(&t_size), sizeof(t_size));
    for (auto t : triangles)
    {
        output.write(reinterpret_cast<char*>(&t.i1_), sizeof(t.i1_));
        output.write(reinterpret_cast<char*>(&t.i2_), sizeof(t.i2_));
        output.write(reinterpret_cast<char*>(&t.i3_), sizeof(t.i3_));
    }

    return true;
}

ShapePtr Shape::read(std::istream& input)
{
    ShapePtr shape(new Shape());

    int p_size = 0;
    input.read(reinterpret_cast<char*>(&p_size), sizeof(p_size));

    for (int i = 0; i < p_size; ++i)
    {
        double x = NAN;
        double y = NAN;
        double z = NAN;
        input.read(reinterpret_cast<char*>(&x), sizeof(x));
        input.read(reinterpret_cast<char*>(&y), sizeof(y));
        input.read(reinterpret_cast<char*>(&z), sizeof(z));
        shape->mesh_.addPoint(x, y, z);
    }

    int t_size = 0;
    input.read(reinterpret_cast<char*>(&t_size), sizeof(t_size));

    for (int i = 0; i < t_size; ++i)
    {
        int i1 = 0;
        int i2 = 0;
        int i3 = 0;
        input.read(reinterpret_cast<char*>(&i1), sizeof(i1));
        input.read(reinterpret_cast<char*>(&i2), sizeof(i2));
        input.read(reinterpret_cast<char*>(&i3), sizeof(i3));
        shape->mesh_.addTriangle(i1, i2, i3);
    }

    return shape;
}

} // namespace geo
