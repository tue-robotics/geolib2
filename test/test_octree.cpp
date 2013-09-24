#include <geolib/Octree.h>
#include <geolib/Box.h>

#include <geolib/sensors/DepthCamera.h>
#include <opencv2/highgui/highgui.hpp>

#include <profiling/Timer.h>

using namespace geo;

cv::Mat depthToRGBImage(const cv::Mat& depth_image, double max_depth) {
    cv::Mat rgb_image(depth_image.rows, depth_image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    for(int y = 0;  y < depth_image.rows; ++y) {
        for(int x = 0; x < depth_image.cols; ++x) {
            float depth = depth_image.at<float>(y, x);

            cv::Vec3b color;
            if (depth == 0) {
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
            } else {
//                float rel_depth = depth / max_depth;
//                int i = rel_depth * 3;
//                float rest = rel_depth * 3 - i;

//                //std::cout << depth << ": " << i << ", " << rest << std::endl;

//                if (i == 0) {
//                    color[0] = 255 * (1 - rest);
//                    color[1] = 255 * rest;
//                    color[2] = 0;
//                } else if (i == 1) {
//                    color[0] = 0;
//                    color[1] = 255 * (1 - rest);
//                    color[2] = 255 * rest;
//                } else if (i == 2) {
//                    color[0] = 0;
//                    color[1] = 0;
//                    color[2] = 255;
//                }
                int c = depth / max_depth * 255;
//                if (c % 10 == 0) {
//                    c = 0;
//                }

                color[0] = c;
                color[1] = c;
                color[2] = c;
            }

            rgb_image.at<cv::Vec3b>(y, x) = color;
        }
    }

    return rgb_image;
}

double render(cv::Mat& image, const Shape& shape, bool rasterize, bool show) {
    Timer timer;
    timer.start();

    DepthCamera cam;

    int N = 0;
    for(double angle = 0; angle < 6.28; angle += 0.1) {
        if (show) {
            image = cv::Mat(image.rows, image.cols, CV_32FC1, 0.0);
        }


        Pose3D pose(0, 0, 3, -1.58, angle, 0);
        if (rasterize) {
            cam.rasterize(shape, pose, image);
        } else {
            cam.render(shape, pose, image);
        }

        if (show) {
            cv::imshow("visualization", depthToRGBImage(image, 8));
            cv::waitKey(3);
        }

        ++N;
    }

    return timer.getElapsedTimeInMilliSec() / N;
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
            for(double z = 0.05; z < 0.75; z += res) {
                table.add(Vector3(0.7 * mx, 0.25 * my, z));
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


    // * * * * * * * * * * * * * * * * * * * *

    cv::Mat image = cv::Mat(480, 640, CV_32FC1, 0.0);

    DepthCamera cam;
    //cam.render(Box(Vector3(-2, -5, -5), Vector3(2, 5, 5)), Pose3D(-2.82, 0, 1.82, 0, 0.5, 0), image);
    Box shape(Vector3(-0.3, -0.5, -0.5), Vector3(0.3, 0.5, 0.5));

    std::string render_type = "raytrace";

//    std::cout << "DepthCamera::raytrace(box):\t" << render(image, shape, false, false) << " ms" << std::endl;
    std::cout << "DepthCamera::rasterize(box):\t" << render(image, shape, true, false) << " ms" << std::endl;
    std::cout << "DepthCamera::rasterize(table):\t" << render(image, table, true, false) << " ms" << std::endl;

    while(true) {
        render(image, table, true, true);



//        std::cout << "Min depth = " << min_depth << std::endl;

//        if (render_type != "") {
//            timer6.stop();
//            std::cout << "DepthCamera::" << render_type << ":\t" << timer6.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
//            if (render_type == "raytrace") {
//                render_type = "rasterize";
//            } else {
//                render_type = "";
//            }
//        }

    }



    return 0;
}
