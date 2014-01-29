#include "geolib/visualization.h"

#include <geolib/Shape.h>
#include <geolib/sensors/DepthCamera.h>

#include <opencv2/highgui/highgui.hpp>

namespace geo {

void visualization::showKinect(ShapeConstPtr shape, double canvas_width, double canvas_height) {
    DepthCamera cam;
    cam.setFocalLengths(554.2559327880068 * canvas_width / 640, 554.2559327880068 * canvas_height / 480);
    cam.setOpticalCenter(320.5 * canvas_width / 640, 240.5 * canvas_height / 480);
    cam.setOpticalTranslation(0, 0);

    double r = shape->getMaxRadius();
    double dist = r / 2;

    for(double angle = 0; angle < 6.283; angle += 0.05) {
        cv::Mat image = cv::Mat(image.rows, image.cols, CV_32FC1, 0.0);

        Pose3D pose(0, 0, dist, -1.57, angle, 0);
        cam.rasterize(*shape, pose, image);

        cv::imshow("visualization", image /  10);
        if (cv::waitKey(30) != -1) {
            return;
        }
    }
}

}

