#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/datatypes.h>
#include <geolib/Mesh.h>
#include <geolib/MeshBVH.h>
#include <geolib/Ray.h>
#include <geolib/Shape.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <random>
#include <thread>
#include <vector>

namespace
{

/**
 * @brief Copy a mesh such that every triangle has its own vertices
 */
geo::Mesh toTriangleSoup(const geo::Mesh& mesh)
{
    geo::Mesh soup;
    for (const auto& t : mesh.getTriangles())
    {
        unsigned int const i1 = soup.addPoint(t.p1());
        unsigned int const i2 = soup.addPoint(t.p2());
        unsigned int const i3 = soup.addPoint(t.p3());
        soup.addTriangle(i1, i2, i3);
    }
    return soup;
}

geo::Shape toShape(const geo::Mesh& mesh)
{
    geo::Shape shape;
    shape.setMesh(mesh);
    return shape;
}

/**
 * @brief UV sphere with radius 1, with shared vertices
 */
geo::Mesh makeSphere(unsigned int n_lat, unsigned int n_lon)
{
    geo::Mesh m;
    m.addPoint(0, 0, 1);
    for (unsigned int i = 1; i < n_lat; ++i)
    {
        for (unsigned int j = 0; j < n_lon; ++j)
        {
            double const th = M_PI * i / n_lat;
            double const ph = 2 * M_PI * j / n_lon;
            m.addPoint(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
        }
    }
    unsigned int const south = m.addPoint(0, 0, -1);

    auto const idx = [n_lon](unsigned int i, unsigned int j) { return 1 + ((i - 1) * n_lon) + (j % n_lon); };
    for (unsigned int j = 0; j < n_lon; ++j)
    {
        m.addTriangle(0, idx(1, j), idx(1, j + 1));
        m.addTriangle(south, idx(n_lat - 1, j + 1), idx(n_lat - 1, j));
    }
    for (unsigned int i = 1; i + 1 < n_lat; ++i)
    {
        for (unsigned int j = 0; j < n_lon; ++j)
        {
            m.addTriangle(idx(i, j), idx(i + 1, j), idx(i + 1, j + 1));
            m.addTriangle(idx(i, j), idx(i + 1, j + 1), idx(i, j + 1));
        }
    }
    return m;
}

double squaredDistanceToBox(const geo::Vector3& p, const geo::Box& box)
{
    double d2 = 0;
    for (unsigned int i = 0; i < 3; ++i)
    {
        double const d = std::max({box.getMin()[i] - p[i], 0.0, p[i] - box.getMax()[i]});
        d2 += d * d;
    }
    return d2;
}

} // namespace

// ----------------------------------------------------------------------------------------------------

class TransformedBoxTest : public testing::TestWithParam<bool>
{
};

TEST_P(TransformedBoxTest, MatchesBox)
{
    bool const soup = GetParam();
    std::mt19937 gen(17);
    std::uniform_real_distribution<double> u(-1, 1);
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);

    for (unsigned int k = 0; k < 20; ++k)
    {
        geo::Box const box(geo::Vector3(-0.3 + (0.1 * u(gen)), -0.2, -0.4),
                           geo::Vector3(0.5, 0.3 + (0.1 * u(gen)), 0.2));
        geo::Pose3D const pose(u(gen), u(gen), u(gen), angle(gen), angle(gen), angle(gen));
        geo::Pose3D const pose_inv = pose.inverse();
        geo::Mesh const mesh = box.getMesh().getTransformed(pose);
        geo::Shape const shape = toShape(soup ? toTriangleSoup(mesh) : mesh);
        ASSERT_TRUE(shape.getMesh().getBVH()->isClosed());

        for (unsigned int i = 0; i < 500; ++i)
        {
            geo::Vector3 const p(2 * u(gen), 2 * u(gen), 2 * u(gen));
            geo::Vector3 const p_box = pose_inv * p;
            ASSERT_EQ(shape.contains(p), box.contains(p_box)) << "k = " << k << ", p = " << p;

            double const radius = 0.3 * (u(gen) + 1);
            double const d = std::sqrt(squaredDistanceToBox(p_box, box));
            if (std::abs(d - radius) > 1e-9)
            {
                ASSERT_EQ(shape.intersect(p, radius), box.intersect(p_box, radius))
                    << "k = " << k << ", p = " << p << ", radius = " << radius;
            }

            // Rays from outside the box, towards a random point
            geo::Vector3 const o = p.normalized() * 5;
            geo::Vector3 const target(0.5 * u(gen), 0.5 * u(gen), 0.5 * u(gen));
            geo::Ray const ray(o, target - o);
            geo::Ray const ray_box(pose_inv * o, pose_inv.getBasis() * ray.getDirection());
            double d_shape = 0;
            double d_box = 0;
            bool const hit_shape = shape.intersect(ray, 0, 100, d_shape);
            bool const hit_box = box.intersect(ray_box, 0, 100, d_box);
            ASSERT_EQ(hit_shape, hit_box) << "k = " << k << ", i = " << i;
            if (hit_box)
            {
                ASSERT_NEAR(d_shape, d_box, 1e-6);
            }
        }
    }
}

INSTANTIATE_TEST_SUITE_P(SharedAndSoup, TransformedBoxTest, testing::Bool());

// ----------------------------------------------------------------------------------------------------

TEST(MeshBVH, Sphere)
{
    unsigned int const n_lat = 32;
    unsigned int const n_lon = 64;
    geo::Shape const sphere = toShape(makeSphere(n_lat, n_lon));
    std::shared_ptr<const geo::MeshBVH> const bvh = sphere.getMesh().getBVH();
    ASSERT_TRUE(bvh->isClosed());
    EXPECT_GT(bvh->numNodes(), 1u);

    // The mesh lies between the inscribed and circumscribed sphere
    double const r_in = std::cos(M_PI / n_lat) * std::cos(M_PI / n_lon);

    std::mt19937 gen(19);
    std::uniform_real_distribution<double> u(-1.2, 1.2);
    for (unsigned int i = 0; i < 5000; ++i)
    {
        geo::Vector3 const p(u(gen), u(gen), u(gen));
        double const r = p.length();
        if (r < r_in)
        {
            ASSERT_TRUE(sphere.contains(p)) << "p = " << p;
        }
        else if (r > 1)
        {
            ASSERT_FALSE(sphere.contains(p)) << "p = " << p;
        }

        if (r > 1)
        {
            // Ray from outside towards the center
            double distance = 0;
            ASSERT_TRUE(sphere.intersect(geo::Ray(p, -p), 0, 10, distance));
            ASSERT_GE(distance, r - 1 - 1e-9);
            ASSERT_LE(distance, r - r_in + 1e-9);
        }
    }
}

TEST(MeshBVH, NearlyCoincidentVertices)
{
    // UV sphere where both poles are rings of separate vertices. sin(pi) is not exactly zero, so the vertices of the
    // south pole differ by about 1e-16.
    unsigned int const n_lat = 16;
    unsigned int const n_lon = 32;
    geo::Mesh mesh;
    for (unsigned int i = 0; i <= n_lat; ++i)
    {
        for (unsigned int j = 0; j < n_lon; ++j)
        {
            double const th = M_PI * i / n_lat;
            double const ph = 2 * M_PI * j / n_lon;
            mesh.addPoint(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
        }
    }
    for (unsigned int i = 0; i < n_lat; ++i)
    {
        for (unsigned int j = 0; j < n_lon; ++j)
        {
            mesh.addTriangle((i * n_lon) + j, ((i + 1) * n_lon) + j, ((i + 1) * n_lon) + ((j + 1) % n_lon));
            mesh.addTriangle((i * n_lon) + j, ((i + 1) * n_lon) + ((j + 1) % n_lon), (i * n_lon) + ((j + 1) % n_lon));
        }
    }

    geo::Shape const shape = toShape(mesh);
    EXPECT_TRUE(shape.getMesh().getBVH()->isClosed());
    EXPECT_TRUE(shape.contains(geo::Vector3(0.1, 0.2, -0.3)));
    EXPECT_FALSE(shape.contains(geo::Vector3(0.1, 0.2, -1.1)));
}

TEST(MeshBVH, OpenMesh)
{
    // Box without its top face (the last two triangles)
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    geo::Mesh open;
    const auto& triangles = box.getMesh().getTriangleIs();
    for (const geo::Vector3& p : box.getMesh().getPoints())
        open.addPoint(p);
    for (std::size_t i = 0; i + 2 < triangles.size(); ++i)
        open.addTriangle(triangles[i].i1_, triangles[i].i2_, triangles[i].i3_);

    geo::Shape const shape = toShape(open);
    EXPECT_FALSE(shape.getMesh().getBVH()->isClosed());
    // The winding number is still well above 0.5 deep inside, and well below outside
    EXPECT_TRUE(shape.contains(geo::Vector3(0, 0, 0)));
    EXPECT_FALSE(shape.contains(geo::Vector3(0, 0, 0.7)));

    // A single triangle does not contain the points around it
    geo::Mesh tri;
    tri.addPoint(-1, -1, 0);
    tri.addPoint(1, -1, 0);
    tri.addPoint(0, 1, 0);
    tri.addTriangle(0, 1, 2);
    geo::Shape const tri_shape = toShape(tri);
    EXPECT_FALSE(tri_shape.contains(geo::Vector3(0, 0, -0.1)));
    EXPECT_FALSE(tri_shape.contains(geo::Vector3(0, 0, 0.1)));
    EXPECT_TRUE(tri_shape.contains(geo::Vector3(0, 0, 0)));
}

TEST(MeshBVH, OverlappingClosedMeshes)
{
    geo::Box const b1(geo::Vector3(0, 0, 0), geo::Vector3(1, 1, 1));
    geo::Box const b2(geo::Vector3(0.5, 0.5, 0.5), geo::Vector3(1.5, 1.5, 1.5));
    geo::Mesh mesh = b1.getMesh();
    mesh.add(b2.getMesh());
    geo::Shape const shape = toShape(mesh);
    ASSERT_TRUE(shape.getMesh().getBVH()->isClosed());

    EXPECT_TRUE(shape.contains(geo::Vector3(0.25, 0.25, 0.25)));
    EXPECT_TRUE(shape.contains(geo::Vector3(0.75, 0.75, 0.75))); // winding number 2
    EXPECT_TRUE(shape.contains(geo::Vector3(1.25, 1.25, 1.25)));
    EXPECT_FALSE(shape.contains(geo::Vector3(1.25, 0.25, 0.25)));
}

TEST(MeshBVH, DistantMesh)
{
    // Quantizing absolute coordinates of a mesh far from the origin would overflow
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    geo::Mesh const mesh = box.getMesh().getTransformed(geo::Pose3D(1e10, 1e10, 1e10));
    EXPECT_TRUE(mesh.getBVH()->isClosed());
    EXPECT_TRUE(mesh.getBVH()->contains(geo::Vector3(1e10, 1e10, 1e10)));

    // Without its last triangle, the mesh is open. An overflow could map all vertices to the same key, which would
    // hide all edges.
    geo::Mesh open;
    for (const geo::Vector3& p : mesh.getPoints())
        open.addPoint(p);
    const auto& triangles = mesh.getTriangleIs();
    for (std::size_t i = 0; i + 1 < triangles.size(); ++i)
        open.addTriangle(triangles[i].i1_, triangles[i].i2_, triangles[i].i3_);
    EXPECT_FALSE(open.getBVH()->isClosed());
}

TEST(MeshBVH, MixedScaleOpenComponent)
{
    // A tiny open triangle collapses to a single vertex when snapping to the grid of the large closed box. Points near
    // it should still be classified by the winding number, which is about 0.35 here.
    geo::Mesh mesh = geo::Box(geo::Vector3(1000, -1, -1), geo::Vector3(1001, 1, 1)).getMesh();
    unsigned int const i1 = mesh.addPoint(0, 0, 0);
    unsigned int const i2 = mesh.addPoint(1e-7, 0, 0);
    unsigned int const i3 = mesh.addPoint(0, 1e-7, 0);
    mesh.addTriangle(i1, i2, i3);

    std::shared_ptr<const geo::MeshBVH> const bvh = mesh.getBVH();
    EXPECT_FALSE(bvh->contains(geo::Vector3(2e-8, 2e-8, -1e-8)));
    EXPECT_FALSE(bvh->contains(geo::Vector3(2e-8, 2e-8, 1e-8)));
    EXPECT_TRUE(bvh->contains(geo::Vector3(1000.5, 0, 0)));
}

TEST(MeshBVH, RayWithTinyDirectionComponent)
{
    geo::Mesh mesh;
    mesh.addPoint(0, 0, 0);
    mesh.addPoint(0, 1, 0);
    mesh.addPoint(0, 0, 1);
    mesh.addTriangle(0, 1, 2);

    double distance = 0;
    ASSERT_TRUE(
        mesh.getBVH()->intersect(geo::Ray(geo::Vector3(-1, -5e-11, 0.25), geo::Vector3(1, 1e-10, 0)), 0, 10, distance));
    EXPECT_NEAR(distance, 1.0, 1e-9);
}

TEST(MeshBVH, OpenMeshOutsideBoundingBox)
{
    // Two copies of the same open triangle have a winding number above 0.5 just above the triangle, which is outside
    // of its (flat) bounding box
    geo::Mesh mesh;
    for (unsigned int k = 0; k < 2; ++k)
    {
        unsigned int const i1 = mesh.addPoint(-1, -1, 0);
        unsigned int const i2 = mesh.addPoint(1, -1, 0);
        unsigned int const i3 = mesh.addPoint(0, 1, 0);
        mesh.addTriangle(i1, i2, i3);
    }
    EXPECT_FALSE(mesh.getBVH()->isClosed());
    EXPECT_TRUE(mesh.getBVH()->contains(geo::Vector3(0, 0, 0.1)));
}

namespace
{

/**
 * @brief Brute force nearest ray-triangle intersection, with the same barycentric tolerance as the hierarchy
 */
bool bruteForceIntersect(const geo::Mesh& mesh, const geo::Ray& r, double& distance)
{
    bool hit = false;
    distance = 1e9;
    const auto& points = mesh.getPoints();
    for (const auto& t : mesh.getTriangleIs())
    {
        geo::Vector3 const v0 = points[t.i1_];
        geo::Vector3 const e1 = points[t.i2_] - v0;
        geo::Vector3 const e2 = points[t.i3_] - v0;
        geo::Vector3 const pvec = r.getDirection().cross(e2);
        double const det = e1.dot(pvec);
        if (det == 0)
            continue;
        geo::Vector3 const tvec = r.getOrigin() - v0;
        double const u = tvec.dot(pvec) / det;
        geo::Vector3 const qvec = tvec.cross(e1);
        double const v = r.getDirection().dot(qvec) / det;
        double const d = e2.dot(qvec) / det;
        if (u >= -1e-9 && v >= -1e-9 && u + v <= 1 + 1e-9 && d > 0 && d < distance)
        {
            distance = d;
            hit = true;
        }
    }
    return hit;
}

} // namespace

TEST(MeshBVH, RaysThroughEdgesAndVertices)
{
    // Many axis aligned boxes, so many leaves of the hierarchy are flat. Rays from far away aimed exactly at vertices,
    // edges and faces should not be lost due to roundoff in the box tests of the hierarchy.
    std::mt19937 gen(29);
    std::uniform_int_distribution<int> grid(-20, 20);
    std::uniform_real_distribution<double> u(-1, 1);
    geo::Mesh mesh;
    std::vector<geo::Vector3> targets;
    for (unsigned int k = 0; k < 200; ++k)
    {
        geo::Vector3 const c(grid(gen) * 0.25, grid(gen) * 0.25, grid(gen) * 0.25);
        geo::Box const box(c - geo::Vector3(0.1, 0.1, 0.1), c + geo::Vector3(0.1, 0.1, 0.1));
        mesh.add(box.getMesh());
        for (const auto& p : box.getMesh().getPoints())
            targets.push_back(p);
        targets.push_back(c + geo::Vector3(0.1, 0, 0));
        targets.push_back(c + geo::Vector3(0.1, 0.1, 0.03));
    }

    std::shared_ptr<const geo::MeshBVH> const bvh = mesh.getBVH();
    for (const geo::Vector3& target : targets)
    {
        for (unsigned int j = 0; j < 10; ++j)
        {
            geo::Vector3 const o = geo::Vector3(u(gen), u(gen), u(gen)).normalized() * (j < 5 ? 50.0 : 5000.0);
            geo::Ray const r(o, target - o);
            double d_bvh = 0;
            double d_brute = 0;
            ASSERT_TRUE(bruteForceIntersect(mesh, r, d_brute));
            ASSERT_TRUE(bvh->intersect(r, 0, 1e9, d_bvh)) << "target = " << target << ", o = " << o;
            ASSERT_NEAR(d_bvh, d_brute, 1e-9 * d_brute) << "target = " << target << ", o = " << o;
        }
    }
}

TEST(MeshBVH, NearlyParallelCrossing)
{
    // Long box, of which one face is tilted only 5e-10 rad from the first direction used by contains. A point just
    // inside of that face leaves the box through it, about 10 m further along the ray.
    geo::Vector3 const d = geo::Vector3(0.3141592, 0.5772156, 0.7536423).normalized();
    geo::Vector3 const u = d.cross(geo::Vector3(0, 0, 1)).normalized();
    geo::Vector3 const w = d.cross(u);
    double const tilt = 5e-10;
    geo::Vector3 const length_dir = (d + (tilt * w)).normalized(); // the box is longer along this direction
    geo::Vector3 const normal = w - (tilt * d); // normal of the tilted face, approximately

    geo::Mesh mesh;
    // Box with corner at the origin, spanned by length_dir (100 m), u (1 m) and normal (1 m)
    for (unsigned int i = 0; i < 8; ++i)
        mesh.addPoint((((i & 1) != 0U) ? 100.0 : 0.0) * length_dir + (((i & 2) != 0U) ? 1.0 : 0.0) * u +
                      (((i & 4) != 0U) ? 1.0 : 0.0) * normal);
    geo::Box const ref(geo::Vector3(0, 0, 0), geo::Vector3(1, 1, 1));
    for (const auto& t : ref.getMesh().getTriangleIs())
        mesh.addTriangle(t.i1_, t.i2_, t.i3_);
    ASSERT_TRUE(mesh.getBVH()->isClosed());

    // 5e-9 inside the tilted face (at 'normal' = 0), near the start of the box
    geo::Vector3 const p = (1.0 * length_dir) + (0.5 * u) + (5e-9 * normal);
    EXPECT_TRUE(mesh.getBVH()->contains(p));
}

TEST(MeshBVH, ZeroAreaTriangleInClosedMesh)
{
    geo::Mesh mesh = geo::Box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5)).getMesh();
    // Zero area triangle with collinear vertices, along an edge of the box (twice, in both orientations)
    unsigned int const i1 = mesh.addPoint(-0.5, -0.5, -0.5);
    unsigned int const i2 = mesh.addPoint(0.0, -0.5, -0.5);
    unsigned int const i3 = mesh.addPoint(0.5, -0.5, -0.5);
    mesh.addTriangle(i1, i2, i3);
    mesh.addTriangle(i3, i2, i1);
    ASSERT_TRUE(mesh.getBVH()->isClosed());

    std::mt19937 gen(31);
    std::uniform_real_distribution<double> u(-0.7, 0.7);
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    for (unsigned int i = 0; i < 2000; ++i)
    {
        geo::Vector3 const p(u(gen), u(gen), u(gen));
        ASSERT_EQ(mesh.getBVH()->contains(p), box.contains(p)) << "p = " << p;
    }
}

TEST(MeshBVH, Empty)
{
    geo::Shape const shape;
    double distance = 0;
    EXPECT_TRUE(shape.getMesh().getBVH()->empty());
    EXPECT_FALSE(shape.contains(geo::Vector3(0, 0, 0)));
    EXPECT_FALSE(shape.intersect(geo::Vector3(0, 0, 0), 1.0));
    EXPECT_FALSE(shape.intersect(geo::Ray(geo::Vector3(-1, 0, 0), geo::Vector3(1, 0, 0)), 0, 10, distance));
}

TEST(MeshBVH, InvalidatedOnChange)
{
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    geo::Mesh mesh = box.getMesh();
    std::shared_ptr<const geo::MeshBVH> const bvh1 = mesh.getBVH();
    EXPECT_EQ(mesh.getBVH(), bvh1); // cached

    geo::Mesh const copy = mesh;
    EXPECT_EQ(copy.getBVH(), bvh1); // shared by copies

    mesh.add(geo::Box(geo::Vector3(2, 2, 2), geo::Vector3(3, 3, 3)).getMesh());
    std::shared_ptr<const geo::MeshBVH> const bvh2 = mesh.getBVH();
    EXPECT_NE(bvh2, bvh1);
    EXPECT_TRUE(bvh2->contains(geo::Vector3(2.5, 2.5, 2.5)));
    EXPECT_FALSE(bvh1->contains(geo::Vector3(2.5, 2.5, 2.5))); // the old hierarchy stays valid
}

TEST(MeshBVH, ConcurrentFirstUse)
{
    geo::Shape const sphere = toShape(makeSphere(32, 64));

    std::atomic<int> wrong{0};
    std::vector<std::thread> threads;
    for (unsigned int k = 0; k < 8; ++k)
    {
        threads.emplace_back(
            [&sphere, &wrong, k]()
            {
                std::mt19937 gen(k);
                std::uniform_real_distribution<double> u(-1.2, 1.2);
                for (unsigned int i = 0; i < 500; ++i)
                {
                    geo::Vector3 const p(u(gen), u(gen), u(gen));
                    double const r = p.length();
                    bool const inside = sphere.contains(p);
                    if ((r < 0.99 && !inside) || (r > 1 && inside))
                        ++wrong;
                }
            });
    }
    for (auto& t : threads)
        t.join();
    EXPECT_EQ(wrong, 0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
