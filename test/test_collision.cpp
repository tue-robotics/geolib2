#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/CompositeShape.h>
#include <geolib/datatypes.h>
#include <geolib/HeightMap.h>
#include <geolib/Mesh.h>
#include <geolib/Ray.h>
#include <geolib/Shape.h>

#include <cmath>
#include <vector>

namespace
{

/**
 * @brief Build a unit box mesh in which every triangle has its own vertices (a "triangle soup"), like many imported
 * meshes have.
 */
geo::Shape makeTriangleSoupBox()
{
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    geo::Mesh mesh;
    for (const auto& t : box.getMesh().getTriangles())
    {
        unsigned int const i1 = mesh.addPoint(t.p1());
        unsigned int const i2 = mesh.addPoint(t.p2());
        unsigned int const i3 = mesh.addPoint(t.p3());
        mesh.addTriangle(i1, i2, i3);
    }
    geo::Shape shape;
    shape.setMesh(mesh);
    return shape;
}

/**
 * @brief A single, open triangle in the z=0 plane, with its normal pointing in +z.
 */
geo::Shape makeSingleTriangle()
{
    geo::Mesh mesh;
    unsigned int const i1 = mesh.addPoint(-1, -1, 0);
    unsigned int const i2 = mesh.addPoint(1, -1, 0);
    unsigned int const i3 = mesh.addPoint(0, 1, 0);
    mesh.addTriangle(i1, i2, i3);
    geo::Shape shape;
    shape.setMesh(mesh);
    return shape;
}

} // namespace

// ----------------------------------------------------------------------------------------------------
//                                            Box
// ----------------------------------------------------------------------------------------------------

TEST(BoxRay, AxisAlignedRayHits)
{
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    double distance = 0;

    ASSERT_TRUE(box.intersect(geo::Ray(geo::Vector3(-5, 0, 0), geo::Vector3(1, 0, 0)), 0, 100, distance));
    EXPECT_NEAR(distance, 4.5, 1e-6);

    ASSERT_TRUE(box.intersect(geo::Ray(geo::Vector3(0, 0, 5), geo::Vector3(0, 0, -1)), 0, 100, distance));
    EXPECT_NEAR(distance, 4.5, 1e-6);

    ASSERT_TRUE(box.intersect(geo::Ray(geo::Vector3(0.2, 7, -0.1), geo::Vector3(0, -1, 0)), 0, 100, distance));
    EXPECT_NEAR(distance, 6.5, 1e-6);
}

TEST(BoxRay, AxisAlignedRayMisses)
{
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    double distance = 0;

    // Parallel to the x-axis, but outside the y-slab
    EXPECT_FALSE(box.intersect(geo::Ray(geo::Vector3(-5, 2, 0), geo::Vector3(1, 0, 0)), 0, 100, distance));
    // Pointing away from the box
    EXPECT_FALSE(box.intersect(geo::Ray(geo::Vector3(-5, 0, 0), geo::Vector3(-1, 0, 0)), 0, 100, distance));
    // Box is beyond the maximum distance
    EXPECT_FALSE(box.intersect(geo::Ray(geo::Vector3(-5, 0, 0), geo::Vector3(1, 0, 0)), 0, 4, distance));
}

TEST(BoxRay, ObliqueRay)
{
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    double distance = 0;

    ASSERT_TRUE(box.intersect(geo::Ray(geo::Vector3(-5, -5, 0), geo::Vector3(1, 1, 0)), 0, 100, distance));
    EXPECT_NEAR(distance, 4.5 * std::sqrt(2), 1e-6);
    EXPECT_FALSE(box.intersect(geo::Ray(geo::Vector3(-5, -4, 0), geo::Vector3(1, -1, 0)), 0, 100, distance));
}

TEST(BoxBox, SeparatedInZ)
{
    // Centers chosen such that c1.y == c2.z, which used to hide a typo in the z-axis test
    geo::Box const b1(geo::Vector3(-0.5, 5, -0.5), geo::Vector3(0.5, 6, 0.5));
    geo::Box const b2(geo::Vector3(-0.5, 5, 5), geo::Vector3(0.5, 6, 6));
    EXPECT_FALSE(b1.intersect(b2));
    EXPECT_FALSE(b2.intersect(b1));

    geo::Box const b3(geo::Vector3(0, 5.5, 0), geo::Vector3(1, 7, 1));
    EXPECT_TRUE(b1.intersect(b3));
}

TEST(BoxMaxRadius, FarthestCorner)
{
    geo::Box const box(geo::Vector3(-2, 0, 0), geo::Vector3(0, 2, 0));
    EXPECT_NEAR(box.getMaxRadius(), std::sqrt(8), 1e-9);

    geo::Box const box2(geo::Vector3(-1, -3, -2), geo::Vector3(4, 1, 1));
    EXPECT_NEAR(box2.getMaxRadius(), std::sqrt(16 + 9 + 4), 1e-9);
}

TEST(BoxEnclose, MeshIsRegenerated)
{
    geo::Box box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    geo::Box const other(geo::Vector3(0, 0, 0), geo::Vector3(1, 1, 1));
    box.enclose(other, geo::Pose3D(1, 0, 0));

    EXPECT_EQ(box.getMesh().getTriangleIs().size(), 12u);
    EXPECT_EQ(box.getMesh().getPoints().size(), 8u);
    EXPECT_NEAR(box.getMax().x, 2.0, 1e-9);
}

// ----------------------------------------------------------------------------------------------------
//                                       Shape (generic mesh)
// ----------------------------------------------------------------------------------------------------

TEST(ShapeSphere, OpenMeshBothSides)
{
    geo::Shape const tri = makeSingleTriangle();

    // In front of the triangle (normal side)
    EXPECT_TRUE(tri.intersect(geo::Vector3(0, 0, 0.5), 0.6));
    EXPECT_FALSE(tri.intersect(geo::Vector3(0, 0, 0.5), 0.4));
    // Behind the triangle
    EXPECT_TRUE(tri.intersect(geo::Vector3(0, 0, -0.5), 0.6));
    EXPECT_FALSE(tri.intersect(geo::Vector3(0, 0, -0.5), 0.4));
    // Close to an edge, but outside of the triangle
    EXPECT_TRUE(tri.intersect(geo::Vector3(0, -1.3, 0), 0.31));
    EXPECT_FALSE(tri.intersect(geo::Vector3(0, -1.3, 0), 0.29));
    // Close to a vertex
    EXPECT_TRUE(tri.intersect(geo::Vector3(0, 1.3, 0), 0.31));
    EXPECT_FALSE(tri.intersect(geo::Vector3(0, 1.3, 0), 0.29));
}

TEST(ShapeSphere, RepeatedVertex)
{
    // Degenerate triangle with a repeated vertex, which collapses to the segment from the origin to (1, 0, 0)
    geo::Mesh mesh;
    mesh.addPoint(0, 0, 0);
    mesh.addPoint(0, 0, 0);
    mesh.addPoint(1, 0, 0);
    mesh.addTriangle(0, 1, 2);
    geo::Shape shape;
    shape.setMesh(mesh);

    EXPECT_TRUE(shape.intersect(geo::Vector3(0.5, 0.1, 0), 0.2));
    EXPECT_FALSE(shape.intersect(geo::Vector3(0.5, 0.3, 0), 0.2));
}

TEST(ShapeContains, TriangleSoupEdgeHit)
{
    geo::Shape const soup = makeTriangleSoupBox();

    // The +x ray from the origin passes exactly through a diagonal edge of the box mesh
    EXPECT_TRUE(soup.contains(geo::Vector3(0, 0, 0)));
    EXPECT_TRUE(soup.contains(geo::Vector3(0.1, 0.2, -0.3)));
    EXPECT_TRUE(soup.contains(geo::Vector3(0.5, 0.5, 0.5)));
    EXPECT_TRUE(soup.contains(geo::Vector3(0, 0.25, 0.5)));
    EXPECT_FALSE(soup.contains(geo::Vector3(0, 0, 0.7)));
    EXPECT_FALSE(soup.contains(geo::Vector3(0.6, 0, 0)));
}

TEST(ShapeContains, MatchesBoxOnGrid)
{
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    geo::Shape shape;
    shape.setMesh(box.getMesh());

    for (double x = -0.75; x <= 0.75; x += 0.125)
        for (double y = -0.75; y <= 0.75; y += 0.125)
            for (double z = -0.75; z <= 0.75; z += 0.125)
            {
                geo::Vector3 const p(x, y, z);
                ASSERT_EQ(shape.contains(p), box.contains(p)) << "p = " << p;
            }
}

TEST(ShapeRay, MeshIntersection)
{
    geo::Shape const soup = makeTriangleSoupBox();
    double distance = 0;

    ASSERT_TRUE(soup.intersect(geo::Ray(geo::Vector3(-5, 0.1, 0.2), geo::Vector3(1, 0, 0)), 0, 100, distance));
    EXPECT_NEAR(distance, 4.5, 1e-6);

    EXPECT_FALSE(soup.intersect(geo::Ray(geo::Vector3(-5, 0.1, 0.2), geo::Vector3(-1, 0, 0)), 0, 100, distance));
    EXPECT_FALSE(soup.intersect(geo::Ray(geo::Vector3(-5, 2, 0.2), geo::Vector3(1, 0, 0)), 0, 100, distance));
    EXPECT_FALSE(soup.intersect(geo::Ray(geo::Vector3(-5, 0.1, 0.2), geo::Vector3(1, 0, 0)), 0, 4, distance));
}

TEST(ShapeRay, DistantRaySmallMesh)
{
    geo::Box const box(geo::Vector3(-0.005, -0.005, -0.005), geo::Vector3(0.005, 0.005, 0.005));
    geo::Shape shape;
    shape.setMesh(box.getMesh());

    double distance = 0;
    ASSERT_TRUE(shape.intersect(geo::Ray(geo::Vector3(1e6, 1e6, 0), geo::Vector3(-1, -1, 0)), 0, 1e7, distance));
    EXPECT_NEAR(distance, (1e6 - 0.005) * std::sqrt(2), 1e-3);
}

TEST(CompositeRay, MeshChild)
{
    geo::CompositeShape comp;
    comp.addShape(makeTriangleSoupBox(), geo::Pose3D(2, 0, 0));
    comp.addShape(geo::Box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5)), geo::Pose3D(5, 0, 0));

    double distance = 0;
    ASSERT_TRUE(comp.intersect(geo::Ray(geo::Vector3(-5, 0, 0.1), geo::Vector3(1, 0, 0)), 0, 100, distance));
    EXPECT_NEAR(distance, 6.5, 1e-6);

    ASSERT_TRUE(comp.intersect(geo::Ray(geo::Vector3(10, 0, 0.1), geo::Vector3(-1, 0, 0)), 0, 100, distance));
    EXPECT_NEAR(distance, 4.5, 1e-6);
}

// ----------------------------------------------------------------------------------------------------
//                                            HeightMap
// ----------------------------------------------------------------------------------------------------

TEST(HeightMapRay, NearestHit)
{
    std::vector<std::vector<double>> grid(4, std::vector<double>(1, 0));
    grid[0][0] = 1;
    grid[3][0] = 1;
    geo::HeightMap const hmap = geo::HeightMap::fromGrid(grid, 1.0);

    double distance = 0;
    ASSERT_TRUE(hmap.intersect(geo::Ray(geo::Vector3(10, 0.5, 0.5), geo::Vector3(-1, 0.01, 0)), 0, 100, distance));
    EXPECT_NEAR(distance, 6.0, 1e-3);

    ASSERT_TRUE(hmap.intersect(geo::Ray(geo::Vector3(-10, 0.5, 0.5), geo::Vector3(1, 0, 0)), 0, 100, distance));
    EXPECT_NEAR(distance, 10.0, 1e-6);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
