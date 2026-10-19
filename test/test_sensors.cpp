#include <gtest/gtest.h>

#include <geolib/Box.h>
#include <geolib/datatypes.h>
#include <geolib/Mesh.h>
#include <geolib/sensors/DepthCamera.h>
#include <geolib/sensors/LaserRangeFinder.h>
#include <geolib/Shape.h>

#include <opencv2/core/core.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

// ----------------------------------------------------------------------------------------------------
//                                          LaserRangeFinder
// ----------------------------------------------------------------------------------------------------

TEST(LRFRender, NoRangeLimit)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-M_PI, M_PI);
    lrf.setNumBeams(9);
    // Range limits are left at 0, which means 'no limit'

    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    std::vector<double> ranges(9, 0);
    lrf.render(box, geo::Pose3D::identity(), geo::Pose3D(5, 0, 0), ranges);

    EXPECT_NEAR(ranges[lrf.getAngleUpperIndex(0.0) - 1], 4.5, 1e-6);
}

TEST(LRFRender, FirstBeamOfLineStartingOutsideView)
{
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-1.0, 1.0);
    lrf.setNumBeams(21); // increment of 0.1 rad

    // A line that starts just below the minimum angle and ends inside the view, oriented such that it faces the
    // sensor
    geo::Vec2 const p1(2 * std::cos(-1.05), 2 * std::sin(-1.05));
    geo::Vec2 const p2(2 * std::cos(-0.5), 2 * std::sin(-0.5));

    std::vector<double> ranges(21, 0);
    lrf.renderLine(p1, p2, ranges);

    EXPECT_GT(ranges[0], 1.9);
    EXPECT_LT(ranges[0], 2.0);
    EXPECT_GT(ranges[4], 1.9);
    EXPECT_LT(ranges[4], 2.0);
    EXPECT_EQ(ranges[7], 0);
}

TEST(LRFRender, AngleRangeBeyondPi)
{
    // Scanner with angles in [0, 2 pi)
    geo::LaserRangeFinder lrf;
    unsigned int const n = 360;
    lrf.setAngleLimits(0, 2 * M_PI * (n - 1) / n);
    lrf.setNumBeams(n);

    // Box behind the scanner, at angle pi + 0.5
    double const a = M_PI + 0.5;
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));
    std::vector<double> ranges(n, 0);
    lrf.render(box, geo::Pose3D::identity(), geo::Pose3D(5 * std::cos(a), 5 * std::sin(a), 0), ranges);

    unsigned int const i = static_cast<unsigned int>(std::round(a / lrf.getAngleIncrement()));
    EXPECT_GT(ranges[i], 4.3);
    EXPECT_LT(ranges[i], 4.6);

    // The cartesian lookup should be consistent with the angle lookup and the rendering
    EXPECT_EQ(lrf.getAngleUpperIndex(std::cos(a), std::sin(a)), lrf.getAngleUpperIndex(a));
    EXPECT_EQ(lrf.getAngleUpperIndex(std::cos(a), std::sin(a)) - 1,
              static_cast<unsigned int>(a / lrf.getAngleIncrement()));
}

TEST(LRFRender, ObjectsAtTheEdgeOfTheAngleRange)
{
    // Scanner with a blind spot behind it
    geo::LaserRangeFinder lrf;
    lrf.setAngleLimits(-2.0, 2.0);
    lrf.setNumBeams(401); // increment of 0.01 rad
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));

    // Boxes at 5 m cover about +/- 0.1 rad. Centered just outside of the angle range, but partly within it.
    for (double const a : {2.08, -2.08})
    {
        std::vector<double> ranges(401, 0);
        lrf.render(box, geo::Pose3D::identity(), geo::Pose3D(5 * std::cos(a), 5 * std::sin(a), 0), ranges);
        unsigned int const i = a > 0 ? 400 : 0;
        EXPECT_GT(ranges[i], 4.4) << "a = " << a;
        EXPECT_LT(ranges[i], 5.0) << "a = " << a;
    }

    // Completely in the blind spot
    for (double const a : {2.3, M_PI, -2.3})
    {
        std::vector<double> ranges(401, 0);
        lrf.render(box, geo::Pose3D::identity(), geo::Pose3D(5 * std::cos(a), 5 * std::sin(a), 0), ranges);
        for (double const r : ranges)
            ASSERT_EQ(r, 0) << "a = " << a;
    }

    // Large object in the blind spot, which reaches into the angle range on both sides
    geo::Box const wall(geo::Vector3(-0.5, -20, -0.5), geo::Vector3(0.5, 20, 0.5));
    std::vector<double> ranges(401, 0);
    lrf.render(wall, geo::Pose3D::identity(), geo::Pose3D(-3, 0, 0), ranges);
    EXPECT_GT(ranges[0], 0);
    EXPECT_GT(ranges[400], 0);
}

// ----------------------------------------------------------------------------------------------------
//                                            DepthCamera
// ----------------------------------------------------------------------------------------------------

namespace
{

/**
 * @brief Double sided rectangle in the plane z = @b z
 */
geo::Shape makeRectangle(double x_min, double y_min, double x_max, double y_max, double z)
{
    geo::Mesh mesh;
    mesh.addPoint(x_min, y_min, z);
    mesh.addPoint(x_max, y_min, z);
    mesh.addPoint(x_max, y_max, z);
    mesh.addPoint(x_min, y_max, z);
    mesh.addTriangle(0, 1, 2);
    mesh.addTriangle(0, 2, 3);
    mesh.addTriangle(0, 2, 1);
    mesh.addTriangle(0, 3, 2);
    geo::Shape shape;
    shape.setMesh(mesh);
    return shape;
}

} // namespace

TEST(DepthCameraRender, FirstColumnAndRow)
{
    geo::DepthCamera const cam(10, 10, 10, 10, 5, 5, 0, 0);

    // Rectangle that projects into pixel column 0 only: u = fx * x / -z + cx in [0.1, 0.9]
    cv::Mat image = cv::Mat::zeros(10, 10, CV_32FC1);
    cam.rasterize(makeRectangle(-0.49, -0.2, -0.41, 0.2, -1.0), geo::Pose3D::identity(), image);
    EXPECT_NEAR(image.at<float>(5, 0), 1.0, 1e-3);

    // Rectangle that projects into pixel row 0 only: v = fy * -y / -z + cy in [0.1, 0.9]
    cv::Mat image2 = cv::Mat::zeros(10, 10, CV_32FC1);
    cam.rasterize(makeRectangle(-0.2, 0.41, 0.2, 0.49, -1.0), geo::Pose3D::identity(), image2);
    EXPECT_NEAR(image2.at<float>(0, 5), 1.0, 1e-3);

    // Rectangle that projects just left of the image: u in [-0.9, -0.1]
    cv::Mat image3 = cv::Mat::zeros(10, 10, CV_32FC1);
    cam.rasterize(makeRectangle(-0.59, -0.2, -0.51, 0.2, -1.0), geo::Pose3D::identity(), image3);
    EXPECT_EQ(cv::countNonZero(image3), 0);
}

TEST(DepthCameraRender, DepthWithinMeshBounds)
{
    geo::DepthCamera const cam(64, 48, 50, 50, 32, 24, 0, 0);

    std::mt19937 gen(42);
    std::uniform_real_distribution<double> xy(-1.5, 1.5);
    std::uniform_real_distribution<double> depth(0.5, 5.0);

    for (unsigned int k = 0; k < 2000; ++k)
    {
        // Random (possibly sliver) triangle in front of the camera, rendered from both sides
        geo::Mesh mesh;
        double z_min = std::numeric_limits<double>::max();
        double z_max = 0;
        for (unsigned int j = 0; j < 3; ++j)
        {
            double const z = depth(gen);
            z_min = std::min(z_min, z);
            z_max = std::max(z_max, z);
            mesh.addPoint(xy(gen) * z, xy(gen) * z, -z);
        }
        mesh.addTriangle(0, 1, 2);
        mesh.addTriangle(0, 2, 1);

        geo::Shape shape;
        shape.setMesh(mesh);

        cv::Mat image = cv::Mat::zeros(48, 64, CV_32FC1);
        cam.rasterize(shape, geo::Pose3D::identity(), image);

        for (int y = 0; y < image.rows; ++y)
        {
            for (int x = 0; x < image.cols; ++x)
            {
                float const d = image.at<float>(y, x);
                if (d == 0)
                    continue;
                ASSERT_TRUE(std::isfinite(d)) << "k = " << k << ", pixel (" << x << ", " << y << ")";
                ASSERT_GE(d, z_min * (1 - 1e-4)) << "k = " << k << ", pixel (" << x << ", " << y << ")";
                ASSERT_LE(d, z_max * (1 + 1e-4)) << "k = " << k << ", pixel (" << x << ", " << y << ")";
            }
        }
    }
}

TEST(DepthCameraRender, TriangleMapResizedOnSizeChange)
{
    geo::Box const box(geo::Vector3(-0.5, -0.5, -3), geo::Vector3(0.5, 0.5, -2));

    geo::PointerMap pointer_map;
    geo::TriangleMap triangle_map;
    int id = 0;

    geo::DepthCamera const cam_small(4, 3, 4, 4, 2, 1.5, 0, 0);
    cv::Mat small = cv::Mat::zeros(3, 4, CV_32FC1);
    cam_small.rasterize(box, geo::Pose3D::identity(), small, pointer_map, &id, triangle_map);

    geo::DepthCamera const cam_large(4, 6, 4, 4, 2, 3, 0, 0);
    cv::Mat large = cv::Mat::zeros(6, 4, CV_32FC1);
    cam_large.rasterize(box, geo::Pose3D::identity(), large, pointer_map, &id, triangle_map);

    ASSERT_EQ(triangle_map.size(), 4u);
    ASSERT_EQ(pointer_map.size(), 4u);
    for (unsigned int x = 0; x < 4; ++x)
    {
        EXPECT_EQ(triangle_map[x].size(), 6u);
        EXPECT_EQ(pointer_map[x].size(), 6u);
    }
}

TEST(DepthCameraRender, DefaultTriangleMapUntouched)
{
    geo::DepthCamera const cam(64, 48, 50, 50, 32, 24, 0, 0);
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));

    cv::Mat image = cv::Mat::zeros(48, 64, CV_32FC1);
    cam.rasterize(box, geo::Pose3D(0, 0, -3), image);
    EXPECT_GT(cv::countNonZero(image), 0);

    // The default triangle map is shared, so it should not be filled when it is not requested
    EXPECT_TRUE(geo::EMPTY_TRIANGLE_MAP.empty());
    EXPECT_TRUE(geo::EMPTY_POINTER_MAP.empty());
}

TEST(DepthCameraRender, OutsideFrustumNotRendered)
{
    geo::DepthCamera const cam(64, 48, 50, 50, 32, 24, 0, 0);
    geo::Box const box(geo::Vector3(-0.5, -0.5, -0.5), geo::Vector3(0.5, 0.5, 0.5));

    // Far to the side, above, below and behind the camera
    for (const geo::Pose3D& pose : {geo::Pose3D(20, 0, -2),
                                    geo::Pose3D(-20, 0, -2),
                                    geo::Pose3D(0, 20, -2),
                                    geo::Pose3D(0, -20, -2),
                                    geo::Pose3D(0, 0, 5)})
    {
        cv::Mat image = cv::Mat::zeros(48, 64, CV_32FC1);
        cam.rasterize(box, pose, image);
        EXPECT_EQ(cv::countNonZero(image), 0);
    }

    // In view
    cv::Mat image = cv::Mat::zeros(48, 64, CV_32FC1);
    cam.rasterize(box, geo::Pose3D(0, 0, -3), image);
    EXPECT_GT(cv::countNonZero(image), 0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
