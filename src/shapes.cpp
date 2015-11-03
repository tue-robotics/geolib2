#include "geolib/shapes.h"

#include "geolib/Mesh.h"
#include "geolib/Shape.h"

namespace geo
{

// ----------------------------------------------------------------------------------------------------

void createCylinder(geo::Shape& shape, double radius, double height, int num_corners)
{
    geo::Mesh mesh;

    // Calculate vertices
    for(int i = 0; i < num_corners; ++i)
    {
        double a = 6.283 * i / num_corners;
        double x = sin(a) * radius;
        double y = cos(a) * radius;

        mesh.addPoint(x, y, -height / 2);
        mesh.addPoint(x, y,  height / 2);
    }

    // Calculate top and bottom triangles
    for(int i = 1; i < num_corners - 1; ++i)
    {
        int i2 = 2 * i;

        // bottom
        mesh.addTriangle(0, i2, i2 + 2);

        // top
        mesh.addTriangle(1, i2 + 3, i2 + 1);
    }

    // Calculate side triangles
    for(int i = 0; i < num_corners; ++i)
    {
        int j = (i + 1) % num_corners;
        mesh.addTriangle(i * 2, i * 2 + 1, j * 2);
        mesh.addTriangle(i * 2 + 1, j * 2 + 1, j * 2);
    }

    shape.setMesh(mesh);
}

// ----------------------------------------------------------------------------------------------------

void createConvexPolygon(geo::Shape& shape, const std::vector<geo::Vec2>& points, double height)
{
    double min_z = -height / 2;
    double max_z =  height / 2;

    geo::Mesh mesh;

    // Add vertices
    for(unsigned int i = 0; i < points.size(); ++i)
    {
        mesh.addPoint(geo::Vector3(points[i].x, points[i].y, min_z));
        mesh.addPoint(geo::Vector3(points[i].x, points[i].y, max_z));
    }

    // Calculate top and bottom triangles
    for(int i = 1; i < points.size() - 1; ++i)
    {
        int i2 = 2 * i;

        // bottom
        mesh.addTriangle(0, i2, i2 + 2);

        // top
        mesh.addTriangle(1, i2 + 3, i2 + 1);
    }

    // Calculate side triangles
    for(int i = 0; i < points.size(); ++i)
    {
        int j = (i + 1) % points.size();
        mesh.addTriangle(i * 2, i * 2 + 1, j * 2);
        mesh.addTriangle(i * 2 + 1, j * 2 + 1, j * 2);
    }

    shape.setMesh(mesh);
}

// ----------------------------------------------------------------------------------------------------

} // end namespace geo

