#ifndef WIRE_DESK_DEPTHCAMERA_H_
#define WIRE_DESK_DEPTHCAMERA_H_

#include <opencv2/core/core.hpp>

#include "geolib/Ray.h"

namespace geo {

class DepthCamera {

public:

    DepthCamera();

    virtual ~DepthCamera();

    void render(const Shape& shape, const Pose3D& pose, cv::Mat& image);

};

}

#endif
