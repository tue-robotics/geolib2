#include "geolib/visualization.h"
#include "geolib/datatypes.h"

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h> // IWYU pragma: keep (needed for the complete type behind ShapeConstPtr)

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace geo
{

void visualization::showKinect(const ShapeConstPtr& shape, double canvas_width, double canvas_height)
{
    DepthCamera const cam(static_cast<uint>(canvas_width),
                          static_cast<uint>(canvas_height),
                          554.2559327880068 * canvas_width / 640,
                          554.2559327880068 * canvas_height / 480,
                          320.5 * canvas_width / 640,
                          240.5 * canvas_height / 480,
                          0,
                          0);

    double const r = shape->getMaxRadius();
    double const dist = r / 2;

    for (int i = 0; i * 0.05 < 6.283; ++i)
    {
        double const angle = i * 0.05;
        cv::Mat image = cv::Mat(cam.height(), cam.width(), CV_32FC1, 0.0);

        Pose3D const pose(0, 0, dist, -1.57, angle, 0);
        cam.rasterize(*shape, pose, image);

        cv::imshow("visualization", image / 10);
        if (cv::waitKey(30) != -1)
        {
            return;
        }
    }
}

} // namespace geo
