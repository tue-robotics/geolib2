#include "geolib/Shape.h"
#include "geolib/Box.h"
#include "geolib/datatypes.h"
#include "geolib/Mesh.h"
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

namespace
{

/**
 * @brief Tolerance used to consider a point to be on the surface of a shape
 */
constexpr double SURFACE_EPS = 1e-9;

/**
 * @brief Tolerance on the barycentric coordinates of a ray-triangle intersection. Without it, roundoff can put a hit on
 * a shared edge or vertex just outside of all triangles sharing it. Accepting hits just outside of a triangle is
 * harmless when looking for the nearest hit.
 */
constexpr double BARYCENTRIC_EPS = 1e-9;

/**
 * @brief Closest point to @b p on the line segment from @b a to @b b
 */
Vector3 closestPointOnSegment(const Vector3& p, const Vector3& a, const Vector3& b)
{
    Vector3 const ab = b - a;
    double const l2 = ab.length2();
    if (l2 == 0)
        return a;
    double const t = std::min(1.0, std::max(0.0, (p - a).dot(ab) / l2));
    return a + t * ab;
}

/**
 * @brief Closest point to @b p on the triangle (a, b, c), including its interior. Determines the Voronoi region of
 * the triangle in which p lies.
 * \a Ericson, C. (2004). Real-Time Collision Detection, section 5.1.5
 */
Vector3 closestPointOnTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
{
    Vector3 const ab = b - a;
    Vector3 const ac = c - a;

    if (ab.cross(ac).length2() == 0)
    {
        // Degenerate (zero area) triangle, e.g. with a repeated vertex. The region tests below would divide by zero, so
        // the closest point is taken from the edges.
        Vector3 const q1 = closestPointOnSegment(p, a, b);
        Vector3 const q2 = closestPointOnSegment(p, b, c);
        Vector3 const q3 = closestPointOnSegment(p, c, a);
        double const l1 = (p - q1).length2();
        double const l2 = (p - q2).length2();
        double const l3 = (p - q3).length2();
        if (l1 <= l2 && l1 <= l3)
            return q1;
        return l2 <= l3 ? q2 : q3;
    }

    // Vertex region of a
    Vector3 const ap = p - a;
    double const d1 = ab.dot(ap);
    double const d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0)
        return a;

    // Vertex region of b
    Vector3 const bp = p - b;
    double const d3 = ab.dot(bp);
    double const d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3)
        return b;

    // Edge region of ab
    double const vc = (d1 * d4) - (d3 * d2);
    if (vc <= 0 && d1 >= 0 && d3 <= 0)
        return a + (d1 / (d1 - d3)) * ab;

    // Vertex region of c
    Vector3 const cp = p - c;
    double const d5 = ab.dot(cp);
    double const d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6)
        return c;

    // Edge region of ac
    double const vb = (d5 * d2) - (d1 * d6);
    if (vb <= 0 && d2 >= 0 && d6 <= 0)
        return a + (d2 / (d2 - d6)) * ac;

    // Edge region of bc
    double const va = (d3 * d6) - (d5 * d4);
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);

    // Face region
    double const denom = va + vb + vc;
    return a + (vb / denom) * ab + (vc / denom) * ac;
}

} // namespace

const std::string Shape::TYPE = "mesh";

Shape::Shape() : bounding_box_cache_valid_(false) {}

Shape::~Shape() = default;

Shape* Shape::clone() const
{
    return new Shape(*this);
}

/**
 * Moller-Trumbore ray-triangle intersection, applied to all triangles of the mesh.
 * \a https://doi.org/10.1080/10867651.1997.10487468
 **/
bool Shape::intersect(const Ray& r, float t0, float t1, double& distance) const
{
    const Mesh& mesh = getMesh();
    const Vector3& o = r.getOrigin();
    const Vector3& dir = r.getDirection(); // unit length

    // Ray should pass the bounding sphere of the mesh. The squared distance of the line to the origin is calculated
    // with the cross product, as |o|^2 - (o dot dir)^2 suffers from cancellation for distant origins.
    if (o.cross(dir).length2() > mesh.getSquaredMaxRadius() * dir.length2())
        return false;

    bool hit = false;
    double t_nearest = t1;

    const std::vector<geo::Vector3>& t_points = mesh.getPoints();
    for (const auto& it : mesh.getTriangleIs())
    {
        const Vector3& v1 = t_points[it.i1_];
        Vector3 const e1 = t_points[it.i2_] - v1;
        Vector3 const e2 = t_points[it.i3_] - v1;

        Vector3 const pvec = dir.cross(e2);
        double const det = e1.dot(pvec);
        if (det == 0)
            // Ray is parallel to the triangle
            continue;

        double const inv_det = 1 / det;
        Vector3 const tvec = o - v1;
        double const u = tvec.dot(pvec) * inv_det;
        if (u < -BARYCENTRIC_EPS || u > 1 + BARYCENTRIC_EPS)
            continue;

        Vector3 const qvec = tvec.cross(e1);
        double const v = dir.dot(qvec) * inv_det;
        if (v < -BARYCENTRIC_EPS || u + v > 1 + BARYCENTRIC_EPS)
            continue;

        double const t = e2.dot(qvec) * inv_det;
        if (t > t0 && t < t_nearest)
        {
            t_nearest = t;
            hit = true;
        }
    }

    if (hit)
        distance = t_nearest;

    return hit;
}

/**
 * The sphere intersects the shape if the distance from its center to the closest point on any of the triangles is at
 * most its radius, or if its center lies within the shape.
 **/
bool Shape::intersect(const Vector3& p, const double RADIUS) const
{
    const Mesh& mesh = getMesh();
    if (p.length() - RADIUS > mesh.getMaxRadius())
    {
        return false;
    }

    if (RADIUS > 0.)
    {
        const double radius2 = RADIUS * RADIUS * (1 + 1e-9); // To prevent any numerical issues
        const std::vector<geo::Vector3>& t_points = mesh.getPoints();
        for (const auto& it : mesh.getTriangleIs())
        {
            const Vector3& v1 = t_points[it.i1_];
            const Vector3& v2 = t_points[it.i2_];
            const Vector3& v3 = t_points[it.i3_];

            // Cheap rejection: the distance to the plane of the triangle is a lower bound of the distance to the
            // triangle. ((p - v1) dot n)^2 / |n|^2 > radius^2
            Vector3 const n = (v2 - v1).cross(v3 - v1);
            double const plane_distance = (p - v1).dot(n);
            if (plane_distance * plane_distance > radius2 * n.length2())
                continue;

            if ((p - closestPointOnTriangle(p, v1, v2, v3)).length2() <= radius2)
                return true;
        }
    }
    return contains(p);
}

/**
 *  The generalized winding number of the mesh around point p is calculated, which is the sum of the signed solid
 *  angles of all triangles, as seen from p, divided by 4 pi. For a closed mesh, it is (-)1 inside and 0 outside. In
 *  contrast to ray casting, this is robust for rays hitting edges or vertices, duplicated vertices and small holes in
 *  the mesh. Points on the surface of the mesh are considered to be inside.
 *  Solid angle of a triangle: \a https://doi.org/10.1109/TBME.1983.325207
 *  Generalized winding number: \a https://doi.org/10.1145/2461912.2461916
 **/
bool Shape::contains(const Vector3& p) const
{
    const Mesh& mesh = getMesh();
    if (p.length2() > mesh.getSquaredMaxRadius() * (1 + 1e-9))
    {
        return false;
    }

    double solid_angle = 0;

    const std::vector<geo::Vector3>& t_points = mesh.getPoints();
    for (const auto& it : mesh.getTriangleIs())
    {
        const Vector3& v1 = t_points[it.i1_];
        const Vector3& v2 = t_points[it.i2_];
        const Vector3& v3 = t_points[it.i3_];

        Vector3 const a = v1 - p;
        Vector3 const b = v2 - p;
        Vector3 const c = v3 - p;

        // det = a dot (b x c) = (v1 - p) dot n, which is proportional to the distance of p to the plane of the triangle
        Vector3 const n = (v2 - v1).cross(v3 - v1);
        double const det = a.dot(n);
        if (det * det <= SURFACE_EPS * SURFACE_EPS * n.length2())
        {
            // p lies in the plane of the triangle. Either it is on the triangle, or the solid angle is zero.
            if ((p - closestPointOnTriangle(p, v1, v2, v3)).length2() <= SURFACE_EPS * SURFACE_EPS)
                return true;
            continue;
        }

        double const la = a.length();
        double const lb = b.length();
        double const lc = c.length();
        double const denom = (la * lb * lc) + (a.dot(b) * lc) + (a.dot(c) * lb) + (b.dot(c) * la);
        solid_angle += 2 * std::atan2(det, denom);
    }

    // Accept both orientations of the triangles (normals pointing outwards or inwards)
    return std::abs(solid_angle) > 2 * std::acos(-1.0); // |winding number| > 0.5
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
