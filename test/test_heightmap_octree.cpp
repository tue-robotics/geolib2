#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/datatypes.h>
#include <geolib/HeightMap.h>
#include <geolib/Octree.h>

#include <algorithm>
#include <random>
#include <vector>

namespace
{

/**
 * @brief Squared distance from @b p to the axis aligned box [@b min, @b max]
 */
double squaredDistance(const geo::Vector3& p, const geo::Vector3& min, const geo::Vector3& max)
{
    double d2 = 0;
    for (unsigned int i = 0; i < 3; ++i)
    {
        double const d = std::max({min[i] - p[i], 0.0, p[i] - max[i]});
        d2 += d * d;
    }
    return d2;
}

class HeightMapTest : public testing::Test
{
protected:
    HeightMapTest() : resolution(0.25), grid(7, std::vector<double>(5, 0))
    {
        std::mt19937 gen(3);
        std::uniform_real_distribution<double> h(-0.5, 1.5);
        for (auto& column : grid)
            for (auto& cell : column)
                cell = std::max(0.0, h(gen)); // Some cells empty
        hmap = geo::HeightMap::fromGrid(grid, resolution);
    }

    /**
     * @brief Reference: a point is inside if it lies in the column of a cell, between 0 and the height of the cell
     */
    bool referenceContains(const geo::Vector3& p) const
    {
        for (unsigned int mx = 0; mx < grid.size(); ++mx)
            for (unsigned int my = 0; my < grid[mx].size(); ++my)
                if (grid[mx][my] > 0 && squaredDistance(p, cellMin(mx, my), cellMax(mx, my)) == 0)
                    return true;
        return false;
    }

    double referenceSquaredDistance(const geo::Vector3& p) const
    {
        double d2 = 1e9;
        for (unsigned int mx = 0; mx < grid.size(); ++mx)
            for (unsigned int my = 0; my < grid[mx].size(); ++my)
                if (grid[mx][my] > 0)
                    d2 = std::min(d2, squaredDistance(p, cellMin(mx, my), cellMax(mx, my)));
        return d2;
    }

    geo::Vector3 cellMin(unsigned int mx, unsigned int my) const { return {mx * resolution, my * resolution, 0}; }

    geo::Vector3 cellMax(unsigned int mx, unsigned int my) const
    {
        return {(mx + 1) * resolution, (my + 1) * resolution, grid[mx][my]};
    }

    double resolution;
    std::vector<std::vector<double>> grid;
    geo::HeightMap hmap;
};

} // namespace

TEST_F(HeightMapTest, Contains)
{
    std::mt19937 gen(5);
    std::uniform_real_distribution<double> x(-0.3, 2.0);
    std::uniform_real_distribution<double> y(-0.3, 1.5);
    std::uniform_real_distribution<double> z(-0.2, 1.7);
    for (unsigned int i = 0; i < 5000; ++i)
    {
        geo::Vector3 const p(x(gen), y(gen), z(gen));
        ASSERT_EQ(hmap.contains(p), referenceContains(p)) << "p = " << p;
    }
}

TEST_F(HeightMapTest, ContainsNearGround)
{
    // The mesh of a height map has no bottom faces, so points close to the ground are the hardest case
    for (unsigned int mx = 0; mx < grid.size(); ++mx)
        for (unsigned int my = 0; my < grid[mx].size(); ++my)
        {
            geo::Vector3 const p((mx + 0.5) * resolution, (my + 0.5) * resolution, 0.01);
            ASSERT_EQ(hmap.contains(p), grid[mx][my] > 0.01) << "p = " << p;
        }
}

TEST_F(HeightMapTest, IntersectSphere)
{
    std::mt19937 gen(7);
    std::uniform_real_distribution<double> x(-0.5, 2.2);
    std::uniform_real_distribution<double> y(-0.5, 1.7);
    std::uniform_real_distribution<double> z(-0.5, 2.0);
    std::uniform_real_distribution<double> r(0.0, 0.3);
    for (unsigned int i = 0; i < 5000; ++i)
    {
        geo::Vector3 const p(x(gen), y(gen), z(gen));
        double const radius = r(gen);
        double const d2 = referenceSquaredDistance(p);
        if (std::abs(d2 - (radius * radius)) < 1e-9)
            continue; // Touching, either answer is fine
        ASSERT_EQ(hmap.intersect(p, radius), d2 <= radius * radius) << "p = " << p << ", radius = " << radius;
    }
}

TEST(HeightMapEmpty, Queries)
{
    geo::HeightMap const hmap = geo::HeightMap::fromGrid(std::vector<std::vector<double>>(2, {0, 0}), 1.0);
    EXPECT_FALSE(hmap.contains(geo::Vector3(0.5, 0.5, 0)));
    EXPECT_FALSE(hmap.intersect(geo::Vector3(0.5, 0.5, 0), 1.0));
}

// ----------------------------------------------------------------------------------------------------

class OctreeTest : public testing::Test
{
protected:
    OctreeTest() : tree(4.0, 0.1)
    {
        std::mt19937 gen(11);
        std::uniform_real_distribution<double> u(-1.5, 1.5);
        for (unsigned int i = 0; i < 300; ++i)
            tree.add(geo::Vector3(u(gen), u(gen), u(gen)));
        tree.getCubes(cubes);
    }

    double referenceSquaredDistance(const geo::Vector3& p) const
    {
        double d2 = 1e9;
        for (const auto& c : cubes)
            d2 = std::min(d2, squaredDistance(p, c.getMin(), c.getMax()));
        return d2;
    }

    geo::Octree tree;
    std::vector<geo::Box> cubes;
};

TEST_F(OctreeTest, IntersectSphere)
{
    std::mt19937 gen(13);
    std::uniform_real_distribution<double> u(-2.5, 2.5);
    std::uniform_real_distribution<double> r(0.0, 0.3);
    for (unsigned int i = 0; i < 2000; ++i)
    {
        geo::Vector3 const p(u(gen), u(gen), u(gen));
        double const radius = r(gen);
        double const d2 = referenceSquaredDistance(p);
        if (std::abs(d2 - (radius * radius)) < 1e-9)
            continue; // Touching, either answer is fine
        ASSERT_EQ(tree.intersect(p, radius), d2 <= radius * radius) << "p = " << p << ", radius = " << radius;
    }
}

TEST_F(OctreeTest, BoundingBox)
{
    geo::Vector3 min(1e9, 1e9, 1e9);
    geo::Vector3 max(-1e9, -1e9, -1e9);
    for (const auto& c : cubes)
        for (unsigned int i = 0; i < 3; ++i)
        {
            min[i] = std::min(min[i], c.getMin()[i]);
            max[i] = std::max(max[i], c.getMax()[i]);
        }

    geo::Box const bb = tree.getBoundingBox();
    for (unsigned int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(bb.getMin()[i], min[i], 1e-9);
        EXPECT_NEAR(bb.getMax()[i], max[i], 1e-9);
    }

    // The bounding box should follow changes of the octree
    tree.add(geo::Vector3(1.95, 1.95, 1.95));
    EXPECT_GT(tree.getBoundingBox().getMax().x, 1.9);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
