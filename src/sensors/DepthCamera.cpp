#include "geolib/sensors/DepthCamera.h"
#include "geolib/Shape.h"

namespace geo {

DepthCamera::DepthCamera() {
}

DepthCamera::~DepthCamera() {

}

void DepthCamera::render(const Shape& shape, const Pose3D& pose, cv::Mat& image) {

    //Transform tf_map_to_sensor(pose);

    tf::Transform pose_in = pose.inverse();

    for(int my = 0; my < image.rows; ++my) {
        for(int mx = 0; mx < image.cols; ++mx) {
            Vector3 dir((double)mx / image.cols - 0.5, (double)my / image.rows - 0.5, 1);
            dir.normalize();

            Ray r(Vector3(0, 0, 0), dir);

            Vector3 dir_transformed = pose_in.getBasis() * r.direction_;
            Ray r_transformed(pose_in.getOrigin(), dir_transformed);

            //std::cout << r_transformed.origin_ << std::endl;

            double distance = 0;
            if (shape.intersect(r_transformed, 0, 10, distance)) {
                if (image.at<float>(my, mx) == 0 || distance < image.at<float>(my, mx)) {
                    image.at<float>(my, mx) = distance;
                }
            }
        }
    }
}

}
