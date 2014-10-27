#include <geolib/Octree.h>
#include <geolib/Box.h>
#include <geolib/HeightMap.h>

#include <geolib/sensors/DepthCamera.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include <opencv2/highgui/highgui.hpp>

#include <profiling/Timer.h>

#include <geolib/Importer.h>

double CANVAS_WIDTH = 640;
double CANVAS_HEIGHT = 480;

using namespace geo;

double renderDepthCamera(cv::Mat& image, const Shape& shape, bool show) {
    DepthCamera cam;
    cam.setFocalLengths(554.2559327880068 * CANVAS_WIDTH / 640, 554.2559327880068 * CANVAS_HEIGHT / 480);
    cam.setOpticalCenter(320.5 * CANVAS_WIDTH / 640, 240.5 * CANVAS_HEIGHT / 480);
    cam.setOpticalTranslation(0, 0);

    Timer timer;
    timer.start();

    int N = 0;
    for(double angle = 0; angle < 6.283; angle += 0.05) {
        if (show) {
            image = cv::Mat(image.rows, image.cols, CV_32FC1, 0.0);
        }


        Pose3D pose(0, -0.5, -5, -1.57, angle, 0);
        cam.rasterize(shape, pose, image);

        if (show) {
            cv::imshow("visualization", image /  10);
            cv::waitKey(30);
        }

        ++N;
    }

    return timer.getElapsedTimeInMilliSec() / N;
}

double renderLRF(cv::Mat& image, const Shape& shape, bool rasterize, bool show) {
    LaserRangeFinder lrf;
    lrf.setAngleLimits(-2, 2);
    lrf.setNumBeams(1000);
    lrf.setRangeLimits(0, 10);

    Timer timer;
    timer.start();

    int N = 0;
    for(double angle = 0; angle < 6.283; angle += 0.05) {
        Pose3D pose(5, 0, -0.5, -1.57, 0, angle);

        std::vector<double> ranges;
        lrf.render(shape, Pose3D(0, 0, 0), pose, ranges);

        if (show) {
            image = cv::Mat(image.rows, image.cols, CV_32FC1, 0.0);

            const std::vector<double>& angles = lrf.getAngles();
            for(unsigned int i = 0; i < angles.size(); ++i) {
                geo::Vector3 p = lrf.polarTo2D(angles[i], ranges[i]);
                double x = (p.x * 25) + image.cols / 2;
                double y = (p.y * 25) + image.rows / 2;

                image.at<float>(y, x) = 1;
            }
            std::cout << std::endl;
            cv::imshow("visualization", image);
            cv::waitKey(30);
        }

        ++N;
    }

    return timer.getElapsedTimeInMilliSec() / N;
}

#include <geolib/serialization.h>
#include <fstream>

int main(int argc, char **argv) {

    serialization::registerDeserializer<Shape>();

    ShapePtr mesh;
    if (argc > 1) {
        std::string filename = argv[1];

        double scale = 1;
        if (argc > 2) {
            scale = atof(argv[2]);
        }

        // first try own file format
        mesh = geo::serialization::fromFile(filename);

        if (!mesh) {
            // If fails, try using assimp
            mesh = geo::Importer::readMeshFile(filename, scale);

            if (!mesh) {
                std::cout << "Could not load " << filename << std::endl;
                return 1;
            }
        }
    }

    if (mesh) {
        geo::serialization::toFile(mesh, "/tmp/test_geolib.geo");
        mesh = geo::serialization::fromFile("/tmp/test_geolib.geo");
    }

    Octree tree(10);

    std::vector<Vector3> points;

    for(double x = -5; x < 5; x += 0.5) {
        for(double y = -5; y < 5; y += 0.5) {
            for(double z = -5; z < 5; z += 0.5) {
                points.push_back(Vector3(x, y, z));
            }
        }
    }

    Timer timer;
    timer.start();
    for(unsigned int i = 0; i < points.size(); ++i) {
        tree.add(points[i]);
    }
    timer.stop();
    std::cout << "Octree::add(Vector3):\t" << timer.getElapsedTimeInMilliSec() / points.size() << " ms" << std::endl;


    Octree table(4);
    double res = table.setResolution(0.1);
    std::cout << "True resolution = " << res << std::endl;

    for(double x = -0.8; x < 0.8; x += res) {
        for(double y = -0.35; y < 0.35; y += res) {
            table.add(Vector3(x, y, 0.75));
        }
    }

    for(int mx = -1; mx <= 1; mx += 2) {
        for(int my = -1; my <= 1; my += 2) {
            if (mx != 1 || my != 1) {
                for(double z = 0.05; z < 0.75; z += res) {
                    table.add(Vector3(0.7 * mx, 0.25 * my, z));
                }
            }
        }
    }

    double distance;
    Ray r(Vector3(0, 0, 5), Vector3(0, 0, -1));

    int N = 10000;
    Timer timer2;
    timer2.start();
    for(int i = 0; i < N; ++i) {
        table.intersect(r, 0, 10, distance);
    }
    timer2.stop();
    std::cout << "Octree::intersect(Ray):\t" << timer2.getElapsedTimeInMilliSec() / N << " ms" << std::endl;

    Box b(Vector3(-0.5, -0.5, 0.5), Vector3(0.5, 0.5, 2));

    N = 10000;
    Timer timer3;
    timer3.start();
    for(int i = 0; i < N; ++i) {
        table.intersect(b);
    }
    timer3.stop();
    std::cout << "Octree::intersect(Box):\t" << timer3.getElapsedTimeInMilliSec() / N << " ms" << std::endl;

    // * * * * * RAYTRACING * * * * * * *

    tree.clear();
    std::vector<Ray> rays;
    for(double y = -2; y < 2; y += tree.getResolution()) {
        for(double x = -2; x < 2; x += tree.getResolution()) {
            Ray r(Vector3(x, y, 5), Vector3(0, 0, -1));
            r.length_ = 5 - x / 2;
            rays.push_back(r);
        }
    }

    Timer timer4;
    timer4.start();
    for(std::vector<Ray>::iterator it = rays.begin(); it != rays.end(); ++it) {
        tree.raytrace(*it, 0, it->length_);
    }
    timer4.stop();
    std::cout << "Octree::raytrace(Ray):\t" << timer4.getElapsedTimeInMilliSec() / rays.size() << " ms" << std::endl;

    rays.clear();
    for(double y = -2; y < 2; y += tree.getResolution()*2) {
        for(double x = -2; x < 2; x += tree.getResolution()*2) {
            Ray r(Vector3(x, y, 5), Vector3(0, 0, -1));
            r.length_ = 5 - y / 2;
            rays.push_back(r);
        }
    }


    Timer timer5;
    timer5.start();
    for(std::vector<Ray>::iterator it = rays.begin(); it != rays.end(); ++it) {
        tree.raytrace(*it, 0, it->length_);
    }
    timer5.stop();
    std::cout << "Octree::raytrace(Ray):\t" << timer5.getElapsedTimeInMilliSec() / rays.size() << " ms" << std::endl;


    Octree axis(2);
    double res2 = table.setResolution(0.1);
    for(double v = 0; v < 1; v += res2) { axis.add(Vector3(v, 0, 0)); }
    for(double v = 0; v < 1; v += res2 * 2) { axis.add(Vector3(0, v, 0)); }
    for(double v = 0; v < 1; v += res2 * 4) { axis.add(Vector3(0, 0, v)); }


    // * * * * * * * * * * * * * * * * * * * *

    cv::Mat image = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_32FC1, 0.0);

//    DepthCamera cam;
    //cam.render(Box(Vector3(-2, -5, -5), Vector3(2, 5, 5)), Pose3D(-2.82, 0, 1.82, 0, 0.5, 0), image);
    Box shape(Vector3(-0.3, -0.5, -0.5), Vector3(0.3, 0.5, 0.5));

//    std::cout << "DepthCamera::raytrace(box):\t" << render(image, shape, false, false) << " ms" << std::endl;

    // Create height map
    int hmap_size = 20;
    std::vector<std::vector<double> > map(hmap_size);
    for(int mx = 0; mx < hmap_size; ++mx) {
        map[mx].resize(hmap_size, 0);
    }

    for(int j = 0; j < hmap_size / 2; ++j) {
        for(int i = 0; i < hmap_size - j * 2; ++i) {
            double h = 0.1 * j;
//            if (j > 0) {
//                h = 0;
//            }

            map[i+j][j] = h;
            map[i+j][hmap_size-j-1] = h;
            map[j][i+j] = h;
            map[hmap_size-j-1][i+j] = h;
        }
    }

    HeightMap hmap = HeightMap::fromGrid(map, 0.1);

    Box plane(Vector3(-10, -10, -0.1), Vector3(10, 10, 0));

    std::cout << "DepthCamera::rasterize(box):\t" << renderDepthCamera(image, shape, false) << " ms" << std::endl;
    std::cout << "DepthCamera::rasterize(table):\t" << renderDepthCamera(image, table, false) << " ms" << std::endl;
    std::cout << "DepthCamera::rasterize(heightmap):\t" << renderDepthCamera(image, hmap, false) << " ms" << std::endl;
//    std::cout << "DepthCamera::rasterize(abstract_shape):\t" << renderDepthCamera(image, tree, true, false) << " ms" << std::endl;

    if (mesh) {
        std::cout << "DepthCamera::rasterize(input_mesh):\t" << renderDepthCamera(image, *mesh, false) << " ms" << std::endl;
    }

    std::cout << "LaserRangeFinder::render(box):\t" << renderLRF(image, shape, true, false) << " ms" << std::endl;
    std::cout << "LaserRangeFinder::render(table):\t" << renderLRF(image, table, true, false) << " ms" << std::endl;
    std::cout << "LaserRangeFinder::render(heightmap):\t" << renderLRF(image, hmap, true, false) << " ms" << std::endl;
//    std::cout << "DepthCamera::rasterize(abstract_shape):\t" << renderLRF(image, tree, true, false) << " ms" << std::endl;

    if (mesh) {
        std::cout << "LaserRangeFinder::rasterize(input_mesh):\t" << renderLRF(image, *mesh, true, false) << " ms" << std::endl;
    }


    DepthCamera cam;
    cam.setFocalLengths(554.2559327880068 * CANVAS_WIDTH / 640, 554.2559327880068 * CANVAS_HEIGHT / 480);
    cam.setOpticalCenter(320.5 * CANVAS_WIDTH / 640, 240.5 * CANVAS_HEIGHT / 480);
    cam.setOpticalTranslation(0, 0);

    LaserRangeFinder lrf;
    lrf.setAngleLimits(-2.3, 2.3);
    lrf.setNumBeams(1000);
    lrf.setRangeLimits(0, 10);

    cv::Mat display_image(CANVAS_HEIGHT, CANVAS_WIDTH * 2, CV_32FC1, 0.0);

    double angle = 0;

    while (true) {
        Shape* shape1 = &hmap;
        if (mesh) {
            shape1 = mesh.get();
        }
        Pose3D pose1(5, 1, -0.5, 0, 0, angle);

        Shape& shape2 = shape;
        Pose3D pose2(5, 2, -0.5, 0, 0.3, angle);

        // * * * * * * DEPTH CAMERA * * * * * *

        cv::Mat depth_image = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_32FC1, 0.0);
        cam.rasterize(*shape1, Pose3D(0, 0, 0, 1.57, 0, -1.57), pose1, depth_image);
        cam.rasterize(shape2, Pose3D(0, 0, 0, 1.57, 0, -1.57), pose2, depth_image);

        cv::Mat depth_image2 = depth_image / 8;
        cv::Mat destinationROI = display_image(cv::Rect(cv::Point(0, 0), cv::Size(CANVAS_WIDTH, CANVAS_HEIGHT)));
        depth_image2.copyTo(destinationROI);

        // * * * * * * LRF * * * * * *

        std::vector<double> ranges;
        lrf.render(*shape1, Pose3D(0, 0, 0), pose1, ranges);
        lrf.render(shape2, Pose3D(0, 0, 0), pose2, ranges);
        lrf.render(Box(Vector3(-0.5, -3.5, -0.5), Vector3(0.5, 3.5, 0.5)), Pose3D(0, 0, 0), geo::Pose3D(-2, 0, 0), ranges);

        cv::Mat lrf_image = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_32FC1, 0.0);

        std::vector<geo::Vector3> points;
        if (lrf.rangesToPoints(ranges, points)) {
            for(unsigned int i = 0; i < points.size(); ++i) {
                const geo::Vector3& p = points[i];
                double x = (-p.y * 25) + image.cols / 2;
                double y = (-p.x * 25) + image.rows / 2;

                if (x >= 0 && y >= 0 && x < image.cols && y < image.rows) {
                    lrf_image.at<float>(y, x) = 1;
                }
            }
        }

        cv::Mat destinationROI2 = display_image(cv::Rect(cv::Point(CANVAS_WIDTH, 0), cv::Size(CANVAS_WIDTH, CANVAS_HEIGHT)));
        lrf_image.copyTo(destinationROI2);

        cv::imshow("visualization", display_image);

        angle += 0.05;

        cv::waitKey(3);
    }

    return 0;
}
