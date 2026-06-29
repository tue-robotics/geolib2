#include "geolib/shapes.h"

#include "geolib/math_types.h"
#include "geolib/Mesh.h"
#include "geolib/Shape.h"

#include <cmath>
#include <vector>

namespace geo
{

// ----------------------------------------------------------------------------------------------------

void createCylinder(geo::Shape& shape, double radius, double height, unsigned int num_corners)
{
    geo::Mesh mesh;

    // Calculate vertices
    for (unsigned int i = 0; i < num_corners; ++i)
    {
        double const a = 2 * M_PI * i / num_corners; // NOLINT(misc-include-cleaner)
        double const x = sin(a) * radius;
        double const y = cos(a) * radius;

        mesh.addPoint(x, y, -height / 2);
        mesh.addPoint(x, y, height / 2);
    }

    // Calculate top and bottom triangles
    for (unsigned int i = 1; i < num_corners - 1; ++i)
    {
        unsigned int const i2 = 2 * i;

        // bottom
        mesh.addTriangle(0, i2, i2 + 2);

        // top
        mesh.addTriangle(1, i2 + 3, i2 + 1);
    }

    // Calculate side triangles
    for (unsigned int i = 0; i < num_corners; ++i)
    {
        unsigned int const j = (i + 1) % num_corners;
        mesh.addTriangle(i * 2, (i * 2) + 1, j * 2);
        mesh.addTriangle((i * 2) + 1, (j * 2) + 1, j * 2);
    }

    shape.setMesh(mesh);
}

// ----------------------------------------------------------------------------------------------------

void createConvexPolygon(geo::Shape& shape, const std::vector<geo::Vec2>& points, double height)
{
    double const min_z = -height / 2;
    double const max_z = height / 2;

    geo::Mesh mesh;

    // Add vertices
    for (const auto& point : points)
    {
        mesh.addPoint(geo::Vector3(point.x, point.y, min_z));
        mesh.addPoint(geo::Vector3(point.x, point.y, max_z));
    }

    // Calculate top and bottom triangles
    for (unsigned int i = 1; i < points.size() - 1; ++i)
    {
        int const i2 = static_cast<int>(2 * i);

        // bottom
        mesh.addTriangle(0, i2, i2 + 2);

        // top
        mesh.addTriangle(1, i2 + 3, i2 + 1);
    }

    // Calculate side triangles
    for (unsigned int i = 0; i < points.size(); ++i)
    {
        int const j = static_cast<int>((i + 1) % points.size());
        mesh.addTriangle(i * 2, (i * 2) + 1, j * 2);
        mesh.addTriangle((i * 2) + 1, (j * 2) + 1, j * 2);
    }

    shape.setMesh(mesh);
}

// ----------------------------------------------------------------------------------------------------

} // end namespace geo
