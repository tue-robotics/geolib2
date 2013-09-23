#ifndef WIRE_DESK_DEPTHCAMERA_H_
#define WIRE_DESK_DEPTHCAMERA_H_

#include <opencv2/core/core.hpp>

#include "geolib/Ray.h"

namespace geo {

class Edge {

public:

    Edge(int x1, int y1, float depth1, int x2, int y2, float depth2) {
        if (y1 < y2) {
            Color1 = depth1;
            X1 = x1;
            Y1 = y1;
            Color2 = depth2;
            X2 = x2;
            Y2 = y2;
        } else {
            Color1 = depth2;
            X1 = x2;
            Y1 = y2;
            Color2 = depth1;
            X2 = x1;
            Y2 = y1;
        }
    }

    int X1, X2;
    int Y1, Y2;
    float Color1, Color2;

};

class Span {

public:

    Span(float color1, int x1, float color2, int x2) {
        if(x1 < x2) {
            Color1 = color1;
            X1 = x1;
            Color2 = color2;
            X2 = x2;
        } else {
            Color1 = color2;
            X1 = x2;
            Color2 = color1;
            X2 = x1;
        }
    }

    int X1, X2;
    float Color1, Color2;

};

class DepthCamera {

public:

    DepthCamera();

    virtual ~DepthCamera();

    void render(const Shape& shape, const Pose3D& pose, cv::Mat& image);   

    bool rasterize(const Shape& shape, const Pose3D& pose, cv::Mat& image);

    cv::Point2d project3Dto2D(const Vector3 p, int width, int height);

protected:

    void drawTriangle(float x1, float y1, float depth1,
                                      float x2, float y2, float depth2,
                                      float x3, float y3, float depth3, cv::Mat& image) const;

    void drawSpansBetweenEdges(const Edge& e1, const Edge& e2, cv::Mat& image) const;

    void drawSpan(const Span &span, int y, cv::Mat& image) const;

};

}

#endif
