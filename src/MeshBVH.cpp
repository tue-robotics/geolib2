#include "geolib/MeshBVH.h"
#include "geolib/datatypes.h"
#include "geolib/Mesh.h"
#include "geolib/Ray.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
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

/**
 * @brief Squared distance from @b p to the axis aligned box [@b min, @b max]
 */
double squaredDistanceToBox(const Vector3& p, const Vector3& min, const Vector3& max)
{
    double d2 = 0;
    for (unsigned int i = 0; i < 3; ++i)
    {
        double const d = std::max({min[i] - p[i], 0.0, p[i] - max[i]});
        d2 += d * d;
    }
    return d2;
}

/**
 * @brief Slab test of a ray against the axis aligned box [@b min, @b max], for distances along the ray in [t0, t1]
 */
bool rayIntersectsBox(
    const Vector3& o, const Vector3& inv_dir, const Vector3& min, const Vector3& max, double t0, double t1)
{
    for (unsigned int i = 0; i < 3; ++i)
    {
        if (std::isinf(inv_dir[i]))
        {
            // Parallel to the slab, explicit check as 0 * inf is undefined
            if (o[i] < min[i] || o[i] > max[i])
                return false;
            continue;
        }
        double const ta = (min[i] - o[i]) * inv_dir[i];
        double const tb = (max[i] - o[i]) * inv_dir[i];
        t0 = std::max(t0, std::min(ta, tb));
        t1 = std::min(t1, std::max(ta, tb));
        if (t0 > t1)
            return false;
    }
    return true;
}

/**
 * @brief Inverse of a direction, with signed infinity for zero components, like Ray does
 */
Vector3 inverseDirection(const Vector3& dir)
{
    Vector3 inv;
    for (unsigned int i = 0; i < 3; ++i)
        inv[i] = dir[i] == 0 ? std::copysign(std::numeric_limits<double>::infinity(), dir[i]) : 1 / dir[i];
    return inv;
}

/**
 * @brief Maximum depth of the hierarchy is about log2(N / MAX_LEAF_SIZE), as nodes are split at the median. This
 * supports any mesh that fits in memory.
 */
constexpr std::size_t MAX_STACK_SIZE = 64;

/**
 * @brief Directions of the rays used to determine whether a point is inside a closed mesh. Not aligned with any axis
 * or diagonal, to make it unlikely that a ray hits an edge or vertex of a mesh made of (axis aligned) boxes.
 */
const std::array<Vector3, 3> RAY_DIRECTIONS = {Vector3(0.3141592, 0.5772156, 0.7536423).normalized(),
                                               Vector3(-0.6931471, 0.2718281, -0.4142135).normalized(),
                                               Vector3(0.1234567, -0.8765432, 0.3819660).normalized()};

/**
 * @brief Relative tolerance on barycentric coordinates and angles, below which a ray crossing is considered degenerate
 */
constexpr double DEGENERATE_EPS = 1e-9;

} // namespace

// ----------------------------------------------------------------------------------------------------

MeshBVH::MeshBVH(const Mesh& mesh) : closed_(false)
{
    const std::vector<Vector3>& points = mesh.getPoints();
    const std::vector<TriangleI>& triangles_i = mesh.getTriangleIs();
    if (triangles_i.empty())
        return;

    // * * * * * * * * Hierarchy * * * * * * * *

    std::vector<Vector3> centroids;
    centroids.reserve(triangles_i.size());
    for (const auto& t : triangles_i)
        centroids.push_back((points[t.i1_] + points[t.i2_] + points[t.i3_]) / 3);

    std::vector<unsigned int> order(triangles_i.size());
    for (unsigned int i = 0; i < order.size(); ++i)
        order[i] = i;

    triangles_.reserve(triangles_i.size());
    for (const auto& t : triangles_i)
        triangles_.push_back({points[t.i1_], points[t.i2_], points[t.i3_]});

    nodes_.reserve(2 * ((triangles_i.size() / MAX_LEAF_SIZE) + 1));
    build(order, centroids, 0, static_cast<unsigned int>(order.size()));

    // Store the triangles in the order of the leaves
    std::vector<Triangle> ordered(triangles_.size());
    for (unsigned int i = 0; i < order.size(); ++i)
        ordered[i] = triangles_[order[i]];
    triangles_.swap(ordered);

    // * * * * * * * * Closedness * * * * * * * *

    // Identify vertices by position instead of index, as many meshes have duplicated vertices. Positions are snapped to
    // a grid relative to the size of the mesh, as duplicated vertices are not always exactly equal. Gaps closed by
    // snapping are far too small to matter for the ray crossings, except for triangles which collapse by snapping;
    // those are marked, so crossings of them are handled as degenerate. If nearly equal vertices end up in different
    // grid cells, the mesh is considered open, which only results in the slower winding number calculation.
    const Vector3& bb_min = nodes_[0].min;
    const Vector3& bb_max = nodes_[0].max;
    double const extent = std::max({bb_max.x - bb_min.x, bb_max.y - bb_min.y, bb_max.z - bb_min.z});
    if (!std::isfinite(extent))
        return;
    double const cell = extent > 0 ? 1e-9 * extent : 1.0;

    // Keys relative to the bounding box of the triangles, so they lie in [0, 1e9] for all vertices used by triangles.
    // Unused vertices are never part of an edge, so their key does not matter.
    std::vector<char> used(points.size(), 0);
    for (const auto& t : triangles_i)
        used[t.i1_] = used[t.i2_] = used[t.i3_] = 1;
    std::vector<std::array<std::int64_t, 3>> keys(points.size(), {-1, -1, -1});
    for (std::size_t i = 0; i < points.size(); ++i)
        if (used[i])
            for (unsigned int k = 0; k < 3; ++k)
                keys[i][k] = static_cast<std::int64_t>(std::llround((points[i][k] - bb_min[k]) / cell));

    std::vector<unsigned int> sorted_points(points.size());
    for (unsigned int i = 0; i < sorted_points.size(); ++i)
        sorted_points[i] = i;
    auto const position_less = [&keys](unsigned int a, unsigned int b) { return keys[a] < keys[b]; };
    std::sort(sorted_points.begin(), sorted_points.end(), position_less);

    std::vector<unsigned int> position_id(points.size());
    unsigned int id = 0;
    for (std::size_t i = 0; i < sorted_points.size(); ++i)
    {
        if (i > 0 && position_less(sorted_points[i - 1], sorted_points[i]))
            ++id;
        position_id[sorted_points[i]] = id;
    }

    // Every undirected edge should be used as often in one direction as in the other
    struct Edge
    {
        unsigned int a, b;
        int direction;
    };
    std::vector<Edge> edges;
    edges.reserve(3 * triangles_i.size());
    std::vector<char> collapsed(triangles_i.size(), 0);
    for (std::size_t i = 0; i < triangles_i.size(); ++i)
    {
        const TriangleI& t = triangles_i[i];
        std::array<unsigned int, 3> const vertices = {t.i1_, t.i2_, t.i3_};
        std::array<unsigned int, 3> const ids = {position_id[t.i1_], position_id[t.i2_], position_id[t.i3_]};
        for (unsigned int j = 0; j < 3; ++j)
        {
            // Two different positions merged by snapping
            if (ids[j] == ids[(j + 1) % 3] && !(points[vertices[j]] == points[vertices[(j + 1) % 3]]))
                collapsed[i] = 1;
        }
        for (unsigned int j = 0; j < 3; ++j)
        {
            unsigned int const from = ids[j];
            unsigned int const to = ids[(j + 1) % 3];
            if (from != to)
                edges.push_back({std::min(from, to), std::max(from, to), from < to ? 1 : -1});
        }
    }
    std::sort(edges.begin(),
              edges.end(),
              [](const Edge& e1, const Edge& e2) { return std::tie(e1.a, e1.b) < std::tie(e2.a, e2.b); });

    closed_ = true;
    for (std::size_t i = 0; i < edges.size() && closed_;)
    {
        int sum = 0;
        std::size_t j = i;
        for (; j < edges.size() && edges[j].a == edges[i].a && edges[j].b == edges[i].b; ++j)
            sum += edges[j].direction;
        closed_ = (sum == 0);
        i = j;
    }

    // In the order of the leaves, like the triangles
    collapsed_.resize(order.size());
    for (unsigned int i = 0; i < order.size(); ++i)
        collapsed_[i] = collapsed[order[i]];
}

// ----------------------------------------------------------------------------------------------------

// NOLINTNEXTLINE(misc-no-recursion)
unsigned int MeshBVH::build(std::vector<unsigned int>& order,
                            const std::vector<Vector3>& centroids,
                            unsigned int begin,
                            unsigned int end)
{
    auto const index = static_cast<unsigned int>(nodes_.size());
    nodes_.push_back({});

    double const inf = std::numeric_limits<double>::infinity();
    Vector3 min(inf, inf, inf);
    Vector3 max(-inf, -inf, -inf);
    Vector3 c_min(inf, inf, inf);
    Vector3 c_max(-inf, -inf, -inf);
    for (unsigned int i = begin; i < end; ++i)
    {
        for (const Vector3& v : triangles_[order[i]])
        {
            for (unsigned int k = 0; k < 3; ++k)
            {
                min[k] = std::min(min[k], v[k]);
                max[k] = std::max(max[k], v[k]);
            }
        }
        const Vector3& c = centroids[order[i]];
        for (unsigned int k = 0; k < 3; ++k)
        {
            c_min[k] = std::min(c_min[k], c[k]);
            c_max[k] = std::max(c_max[k], c[k]);
        }
    }
    nodes_[index].min = min;
    nodes_[index].max = max;

    // Split along the axis with the largest spread of the triangle centroids
    Vector3 const extent = c_max - c_min;
    unsigned int axis = 0;
    if (extent.y > extent[axis])
        axis = 1;
    if (extent.z > extent[axis])
        axis = 2;

    if (end - begin <= MAX_LEAF_SIZE || extent[axis] <= 0)
    {
        nodes_[index].first = begin;
        nodes_[index].count = end - begin;
        return index;
    }

    unsigned int const mid = begin + ((end - begin) / 2);
    std::nth_element(order.begin() + begin,
                     order.begin() + mid,
                     order.begin() + end,
                     [&centroids, axis](unsigned int a, unsigned int b)
                     { return centroids[a][axis] < centroids[b][axis]; });

    build(order, centroids, begin, mid);
    unsigned int const second = build(order, centroids, mid, end);
    nodes_[index].first = second;
    nodes_[index].count = 0;
    return index;
}

// ----------------------------------------------------------------------------------------------------

bool MeshBVH::intersect(const Ray& r, double t0, double t1, double& distance) const
{
    if (nodes_.empty())
        return false;

    const Vector3& o = r.getOrigin();
    const Vector3& dir = r.getDirection();
    Vector3 const inv_dir = inverseDirection(dir);

    bool hit = false;
    double t_nearest = t1;

    std::array<unsigned int, MAX_STACK_SIZE> stack{};
    std::size_t stack_size = 0;
    stack[stack_size++] = 0;
    while (stack_size > 0)
    {
        const Node& node = nodes_[stack[--stack_size]];
        if (!rayIntersectsBox(o, inv_dir, node.min, node.max, t0, t_nearest))
            continue;

        if (node.count == 0)
        {
            stack[stack_size++] = node.first;
            stack[stack_size++] = static_cast<unsigned int>(&node - nodes_.data()) + 1;
            continue;
        }

        // Moller-Trumbore ray-triangle intersection, https://doi.org/10.1080/10867651.1997.10487468
        for (unsigned int i = node.first; i < node.first + node.count; ++i)
        {
            const Triangle& t = triangles_[i];
            Vector3 const e1 = t[1] - t[0];
            Vector3 const e2 = t[2] - t[0];

            Vector3 const pvec = dir.cross(e2);
            double const det = e1.dot(pvec);
            if (det == 0)
                // Ray is parallel to the triangle
                continue;

            double const inv_det = 1 / det;
            Vector3 const tvec = o - t[0];
            double const u = tvec.dot(pvec) * inv_det;
            if (u < 0 || u > 1)
                continue;

            Vector3 const qvec = tvec.cross(e1);
            double const v = dir.dot(qvec) * inv_det;
            if (v < 0 || u + v > 1)
                continue;

            double const t_hit = e2.dot(qvec) * inv_det;
            if (t_hit > t0 && t_hit < t_nearest)
            {
                t_nearest = t_hit;
                hit = true;
            }
        }
    }

    if (hit)
        distance = t_nearest;

    return hit;
}

// ----------------------------------------------------------------------------------------------------

bool MeshBVH::intersectSurface(const Vector3& p, double radius) const
{
    if (nodes_.empty() || radius < 0)
        return false;

    double const radius2 = radius * radius * (1 + 1e-9); // To prevent any numerical issues

    std::array<unsigned int, MAX_STACK_SIZE> stack{};
    std::size_t stack_size = 0;
    stack[stack_size++] = 0;
    while (stack_size > 0)
    {
        const Node& node = nodes_[stack[--stack_size]];
        if (squaredDistanceToBox(p, node.min, node.max) > radius2)
            continue;

        if (node.count == 0)
        {
            stack[stack_size++] = node.first;
            stack[stack_size++] = static_cast<unsigned int>(&node - nodes_.data()) + 1;
            continue;
        }

        for (unsigned int i = node.first; i < node.first + node.count; ++i)
        {
            const Triangle& t = triangles_[i];

            // Cheap rejection: the distance to the plane of the triangle is a lower bound of the distance to the
            // triangle. ((p - v1) dot n)^2 / |n|^2 > radius^2
            Vector3 const n = (t[1] - t[0]).cross(t[2] - t[0]);
            double const plane_distance = (p - t[0]).dot(n);
            if (plane_distance * plane_distance > radius2 * n.length2())
                continue;

            if ((p - closestPointOnTriangle(p, t[0], t[1], t[2])).length2() <= radius2)
                return true;
        }
    }

    return false;
}

// ----------------------------------------------------------------------------------------------------

bool MeshBVH::contains(const Vector3& p) const
{
    if (nodes_.empty())
        return false;

    // Outside of the bounding box of a closed mesh, the winding number is zero. For an open mesh, it might not be.
    if (closed_ && squaredDistanceToBox(p, nodes_[0].min, nodes_[0].max) > SURFACE_EPS * SURFACE_EPS)
        return false;

    // Points on the surface are considered to be inside
    if (intersectSurface(p, SURFACE_EPS))
        return true;

    if (closed_)
    {
        // For a closed mesh, the number of signed crossings of any ray from p is equal to the winding number. Try
        // another direction in case of a degenerate crossing.
        for (const Vector3& dir : RAY_DIRECTIONS)
        {
            int count = 0;
            if (countCrossings(p, dir, count))
                return count != 0;
        }
    }

    // Accept both orientations of the triangles (normals pointing outwards or inwards)
    return windingNumber(p) > 0.5;
}

// ----------------------------------------------------------------------------------------------------

bool MeshBVH::countCrossings(const Vector3& p, const Vector3& dir, int& count) const
{
    Vector3 const inv_dir = inverseDirection(dir);
    double const inf = std::numeric_limits<double>::infinity();
    count = 0;

    std::array<unsigned int, MAX_STACK_SIZE> stack{};
    std::size_t stack_size = 0;
    stack[stack_size++] = 0;
    while (stack_size > 0)
    {
        const Node& node = nodes_[stack[--stack_size]];
        if (!rayIntersectsBox(p, inv_dir, node.min, node.max, 0, inf))
            continue;

        if (node.count == 0)
        {
            stack[stack_size++] = node.first;
            stack[stack_size++] = static_cast<unsigned int>(&node - nodes_.data()) + 1;
            continue;
        }

        for (unsigned int i = node.first; i < node.first + node.count; ++i)
        {
            const Triangle& t = triangles_[i];
            Vector3 const e1 = t[1] - t[0];
            Vector3 const e2 = t[2] - t[0];
            Vector3 const tvec = p - t[0];

            Vector3 const pvec = dir.cross(e2);
            double const det = e1.dot(pvec); // = -dir dot (e1 x e2)
            double const n_length = e1.cross(e2).length();
            if (std::abs(det) <= DEGENERATE_EPS * n_length)
            {
                // Ray (almost) parallel to the triangle. Only a problem if the ray (almost) lies in its plane.
                if (n_length == 0 || std::abs(tvec.dot(e1.cross(e2))) <= DEGENERATE_EPS * n_length)
                    return false;
                continue;
            }

            double const inv_det = 1 / det;
            double const u = tvec.dot(pvec) * inv_det;
            Vector3 const qvec = tvec.cross(e1);
            double const v = dir.dot(qvec) * inv_det;
            if (u < -DEGENERATE_EPS || v < -DEGENERATE_EPS || u + v > 1 + DEGENERATE_EPS)
                continue;

            double const t_hit = e2.dot(qvec) * inv_det;
            if (t_hit <= 0)
                continue;

            if (u < DEGENERATE_EPS || v < DEGENERATE_EPS || u + v > 1 - DEGENERATE_EPS)
                // Crossing (almost) on an edge or vertex, which might be counted zero or multiple times
                return false;

            if (collapsed_[i])
                // Triangle ignored in the closedness check, so the crossing count might not be the winding number
                return false;

            count += det > 0 ? 1 : -1;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

/**
 *  The generalized winding number of the mesh around point p is the sum of the signed solid angles of all triangles,
 *  as seen from p, divided by 4 pi. For a closed mesh, it is (-)1 inside and 0 outside. It is robust for rays hitting
 *  edges or vertices, duplicated vertices and small holes in the mesh.
 *  Solid angle of a triangle: \a https://doi.org/10.1109/TBME.1983.325207
 *  Generalized winding number: \a https://doi.org/10.1145/2461912.2461916
 **/
double MeshBVH::windingNumber(const Vector3& p) const
{
    double solid_angle = 0;
    for (const Triangle& t : triangles_)
    {
        Vector3 const a = t[0] - p;
        Vector3 const b = t[1] - p;
        Vector3 const c = t[2] - p;

        // det = a dot (b x c) = (v1 - p) dot n, which is proportional to the distance of p to the plane of the triangle
        Vector3 const n = (t[1] - t[0]).cross(t[2] - t[0]);
        double const det = a.dot(n);
        if (det * det <= SURFACE_EPS * SURFACE_EPS * n.length2())
            // p lies in the plane of the triangle, but not on the triangle (checked before), so the solid angle is zero
            continue;

        double const la = a.length();
        double const lb = b.length();
        double const lc = c.length();
        double const denom = (la * lb * lc) + (a.dot(b) * lc) + (a.dot(c) * lb) + (b.dot(c) * la);
        solid_angle += 2 * std::atan2(det, denom);
    }

    return std::abs(solid_angle) / (2 * std::acos(-1.0) * 2);
}

} // namespace geo
