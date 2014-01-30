#include <geolib/Importer.h>
#include <geolib/serialization.h>
#include <geolib/sensors/DepthCamera.h>

#include <opencv2/highgui/highgui.hpp>

double CANVAS_WIDTH = 640;
double CANVAS_HEIGHT = 480;

int main(int argc, char **argv) {

    if (argc != 2) {
        std::cout << "Usage: show INPUT_FILE" << std::endl;
        return 1;
    }

    geo::serialization::registerDeserializer<geo::Shape>();

    std::string filename = argv[1];

    // first try own file format
    geo::ShapePtr shape = geo::serialization::fromFile(filename);

    if (!shape) {
        // If fails, try using assimp
        shape = geo::Importer::readMeshFile(filename);

        if (!shape) {
            std::cout << "Could not load " << argv[1] << std::endl;
            return 1;
        }
    }

    geo::DepthCamera cam;
    cam.setFocalLengths(554.2559327880068 * CANVAS_WIDTH / 640, 554.2559327880068 * CANVAS_HEIGHT / 480);
    cam.setOpticalCenter(320.5 * CANVAS_WIDTH / 640, 240.5 * CANVAS_HEIGHT / 480);
    cam.setOpticalTranslation(0, 0);

    double r = shape->getMaxRadius();
    double dist = r * 3;

    double angle = 0;

    while (true) {
        geo::Pose3D pose(dist, 0, 0, 0, 0, angle);

        // * * * * * * DEPTH CAMERA * * * * * *

        cv::Mat depth_image = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_32FC1, 0.0);
        cam.rasterize(*shape, geo::Pose3D(0, 0, 0, 1.57, 0, -1.57), pose, depth_image);

        for(int y = 0; y < depth_image.rows; ++y) {
            for(int x = 0; x < depth_image.cols; ++x) {
                float d = depth_image.at<float>(y, x);
                if (d > 0) {
                    depth_image.at<float>(y, x) = (d - dist + r) / (2 * r);
                }
            }
        }

        cv::imshow("visualization", depth_image);
        cv::waitKey(10);

        angle += 0.07;
    }

    return 0;
}
