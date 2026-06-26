#include "geolib/datatypes.h"
#include <geolib/io/import.h>
#include <geolib/sensors/DepthCamera.h>
#include <geolib/serialization.h>
#include <geolib/Shape.h>

#include <iostream>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ostream>
#include <string>

namespace
{
const double CANVAS_WIDTH = 640;
const double CANVAS_HEIGHT = 480;
} // namespace

int main(int argc, char** argv)
{

    if (argc != 2)
    {
        std::cout << "Usage: show INPUT_FILE" << '\n';
        return 1;
    }

    geo::serialization::registerDeserializer<geo::Shape>();

    std::string const filename = argv[1];

    // first try own file format
    geo::ShapePtr shape = geo::serialization::fromFile(filename);

    if (!shape)
    {
        // If fails, try using assimp
        shape = geo::io::readMeshFile(filename);

        if (!shape)
        {
            std::cout << "Could not load " << argv[1] << '\n';
            return 1;
        }
    }

    geo::DepthCamera const cam(static_cast<uint>(CANVAS_WIDTH),
                               static_cast<uint>(CANVAS_HEIGHT),
                               554.2559327880068 * CANVAS_WIDTH / 640,
                               554.2559327880068 * CANVAS_HEIGHT / 480,
                               320.5 * CANVAS_WIDTH / 640,
                               240.5 * CANVAS_HEIGHT / 480,
                               0,
                               0);

    double const r = shape->getMaxRadius();
    double const dist = r * 3;

    double angle = 0;

    while (true)
    {
        geo::Pose3D const pose(dist, 0, 0, 0, 0, angle);

        // * * * * * * DEPTH CAMERA * * * * * *

        cv::Mat depth_image = cv::Mat(static_cast<int>(CANVAS_HEIGHT), static_cast<int>(CANVAS_WIDTH), CV_32FC1, 0.0);
        cam.rasterize(*shape, geo::Pose3D(0, 0, 0, 1.57, 0, -1.57), pose, depth_image);

        for (int y = 0; y < depth_image.rows; ++y)
        {
            for (int x = 0; x < depth_image.cols; ++x)
            {
                float const d = depth_image.at<float>(y, x);
                if (d > 0)
                {
                    depth_image.at<float>(y, x) = static_cast<float>((d - dist + r) / (2 * r));
                }
            }
        }

        cv::imshow("visualization", depth_image);
        char const key = static_cast<char>(cv::waitKey(10));
        if (key == 'q')
        {
            break;
        }

        angle += 0.07;
    }

    return 0;
}
