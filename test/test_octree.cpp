#include <geolib/Octree.h>
#include <geolib/Box.h>
#include <geolib/HeightMap.h>

#include <geolib/sensors/DepthCamera.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include <opencv2/highgui/highgui.hpp>

#include <profiling/Timer.h>

using namespace geo;

double renderDepthCamera(cv::Mat& image, const Shape& shape, bool rasterize, bool show) {
    Timer timer;
    timer.start();

    DepthCamera cam;
    cam.setFocalLengths(554.2559327880068, 554.2559327880068);
    cam.setOpticalCenter(320.5, 240.5);
    cam.setOpticalTranslation(0, 0);

    int N = 0;
    for(double angle = 0; angle < 6.283; angle += 0.05) {
        if (show) {
            image = cv::Mat(image.rows, image.cols, CV_32FC1, 0.0);
        }


        Pose3D pose(0, -0.5, -5, -1.57, angle, 0);
        if (rasterize) {
            cam.rasterize(shape, pose, image);
        } else {
            cam.render(shape, pose, image);
        }

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

    for(double angle = 0; angle < 6.283; angle += 0.05) {
        Pose3D pose(5, 0, -0.5, -1.57, 0, angle);

        std::vector<double> ranges;
        std::cout << "rendering..." << std::endl;
        lrf.render(shape, Pose3D(0, 0, 0), pose, ranges);
        std::cout << "..done" << std::endl;

        if (show) {
            image = cv::Mat(image.rows, image.cols, CV_32FC1, 0.0);

            const std::vector<double>& angles = lrf.getAngles();
            for(unsigned int i = 0; i < angles.size(); ++i) {
                geo::Vector3 p = lrf.polarTo2D(angles[i], ranges[i]);
                double x = (p.x() * 25) + image.cols / 2;
                double y = (p.y() * 25) + image.rows / 2;


                image.at<float>(y, x) = 1;
                std::cout << ranges[i] << " ";
            }
            std::cout << std::endl;
            cv::imshow("visualization", image);
            cv::waitKey(30);
        }

    }

    return 0;
}

int main(int argc, char **argv) {

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

    cv::Mat image = cv::Mat(480, 640, CV_32FC1, 0.0);

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
            map[i+j][j] = j * 0.1;
            map[i+j][hmap_size-j-1] = j * 0.1;
            map[j][i+j] = j * 0.1;
            map[hmap_size-j-1][i+j] = j * 0.1;
        }
    }

    HeightMap hmap = HeightMap::fromGrid(map, 0.1);

    Box plane(Vector3(-10, -10, -0.1), Vector3(10, 10, 0));

    std::cout << "DepthCamera::rasterize(box):\t" << renderDepthCamera(image, shape, true, false) << " ms" << std::endl;
    std::cout << "DepthCamera::rasterize(table):\t" << renderDepthCamera(image, table, true, false) << " ms" << std::endl;
    std::cout << "DepthCamera::rasterize(heightmap):\t" << renderDepthCamera(image, hmap, true, false) << " ms" << std::endl;
    //std::cout << "DepthCamera::rasterize(abstract_shape):\t" << render(image, tree, true, false) << " ms" << std::endl;

    DepthCamera cam;
    cam.setFocalLengths(554.2559327880068, 554.2559327880068);
    cam.setOpticalCenter(320.5, 240.5);
    cam.setOpticalTranslation(0, 0);

    LaserRangeFinder lrf;
    lrf.setAngleLimits(-2, 2);
    lrf.setNumBeams(1000);
    lrf.setRangeLimits(0, 10);

    double angle = 0;
    while (true) {
        Pose3D pose(5, 0, -0.5, 0, 0, angle);

        // * * * * * * DEPTH CAMERA * * * * * *
        cv::Mat depth_image = cv::Mat(480, 640, CV_32FC1, 0.0);

        cam.rasterize(table, Pose3D(0, 0, 0, 1.57, 0, -1.57), pose, depth_image);
        cv::imshow("camera", depth_image / 8);

        // * * * * * * LRF * * * * * *

        std::vector<double> ranges;
        lrf.render(table, Pose3D(0, 0, 0), pose, ranges);

        cv::Mat lrf_image = cv::Mat(480, 640, CV_32FC1, 0.0);
        const std::vector<double>& angles = lrf.getAngles();
        for(unsigned int i = 0; i < angles.size(); ++i) {
            geo::Vector3 p = lrf.polarTo2D(angles[i], ranges[i]);
            double x = (p.x() * 25) + image.cols / 2;
            double y = (p.y() * 25) + image.rows / 2;
            lrf_image.at<float>(y, x) = 1;
            std::cout << ranges[i] << " ";
        }
        std::cout << std::endl;
        cv::imshow("lrf", lrf_image);

        angle += 0.05;

        cv::waitKey(30);
    }

    return 0;
}
