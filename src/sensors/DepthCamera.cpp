#include "geolib/sensors/DepthCamera.h"
#include "geolib/Shape.h"

namespace geo {

DepthCamera::DepthCamera() {
}

DepthCamera::~DepthCamera() {

}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        RAYTRACING
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

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

void DepthCamera::setFocalLengths(double fx, double fy) {
    fx_ = fx;
    fy_ = fy;
}

void DepthCamera::setOpticalCenter(double cx, double cy) {
    cx_ = cx;
    cy_ = cy;
}

void DepthCamera::setOpticalTranslation(double tx, double ty) {
    tx_ = tx;
    ty_ = ty;
}

cv::Point2d DepthCamera::project3Dto2D(const Vector3 p, int width, int height) {
    //std::cout << -p.z() << std::endl;
    //return cv::Point2d((p.x() / -p.z() + 0.5) * width, (-p.y() / -p.z() + 0.5) * height);

//    cv::Point2d uv_rect;
//    uv_rect.x = (fx()*xyz.x + Tx()) / xyz.z + cx();
//    uv_rect.y = (fy()*xyz.y + Ty()) / xyz.z + cy();
//    return uv_rect;

    return cv::Point2d((fx_ * p.x() + tx_) / -p.z() + cx_, (fy_ * -p.y() + ty_) / -p.z() + cy_);
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        RASTERIZATION
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


bool DepthCamera::rasterize(const Shape& shape, const Pose3D& pose, cv::Mat& image) {

    tf::Transform pose_in = pose;
    //pose_in.setOrigin(-pose.getOrigin());
    //tf::Transform pose_in = Pose3D(0, 0, -5, 2.3, 0.3, 0.3);//pose.inverse();

    std::vector<Triangle> triangles = shape.getMesh();

    for(std::vector<Triangle>::const_iterator it_tri = triangles.begin(); it_tri != triangles.end(); ++it_tri) {
        const Triangle& t = *it_tri;

        Vector3 p1_3d = pose_in * t.p1_;
        Vector3 p2_3d = pose_in * t.p2_;
        Vector3 p3_3d = pose_in * t.p3_;

        cv::Point2d p1_2d = project3Dto2D(p1_3d, image.cols, image.rows);
        cv::Point2d p2_2d = project3Dto2D(p2_3d, image.cols, image.rows);
        cv::Point2d p3_2d = project3Dto2D(p3_3d, image.cols, image.rows);

//        std::cout << std::endl;
//        std::cout << t.p1_.x() << ", " << t.p1_.y() << ", " << t.p1_.z() << std::endl;
//        std::cout << t.p2_.x() << ", " << t.p2_.y() << ", " << t.p2_.z() << std::endl;
//        std::cout << t.p3_.x() << ", " << t.p3_.y() << ", " << t.p3_.z() << std::endl;
//        std::cout << "---" << std::endl;

//        std::cout << p1_3d.x() << ", " << p1_3d.y() << ", " << p1_3d.z() << std::endl;
//        std::cout << p2_3d.x() << ", " << p2_3d.y() << ", " << p2_3d.z() << std::endl;
//        std::cout << p3_3d.x() << ", " << p3_3d.y() << ", " << p3_3d.z() << std::endl;
//        std::cout << "---" << std::endl;

//        std::cout << p1_2d << std::endl;
//        std::cout << p2_2d << std::endl;
//        std::cout << p3_2d << std::endl;
//        std::cout << "---" << std::endl;

//        if (p1_2d.x >= -1000 && p1_2d.x < 6400 && p1_2d.y >= -1000 && p1_2d.y < 4800
//                && p2_2d.x >= -1000 && p2_2d.x < 6400 && p2_2d.y >= -1000 && p2_2d.y < 4800
//                && p3_2d.x >= -1000 && p3_2d.x < 6400 && p3_2d.y >= -1000 && p3_2d.y < 4800) {

//            std::cout << "DRAW!" << std::endl;

        if ((p1_2d.x >= 0 && p1_2d.x < image.cols && p1_2d.y >= 0 && p1_2d.y < image.rows)
            || (p2_2d.x >= 0 && p2_2d.x < image.cols && p2_2d.y >= 0 && p2_2d.y < image.rows)
            || (p3_2d.x >= 0 && p3_2d.x < image.cols && p3_2d.y >= 0 && p3_2d.y < image.rows)) {

            drawTriangle(p1_2d.x, p1_2d.y, -p1_3d.z(),
                         p2_2d.x, p2_2d.y, -p2_3d.z(),
                         p3_2d.x, p3_2d.y, -p3_3d.z(), image);



        }
    }

    for(int y = 0; y < image.rows; ++y) {
        for(int x = 0; x < image.cols; ++x) {
            if (image.at<float>(y, x) > 9) {
                image.at<float>(y, x) = 0.0 / 0.0;
            }
        }
    }

    return true;
}

void DepthCamera::drawTriangle(float x1, float y1, float depth1,
                                  float x2, float y2, float depth2,
                                  float x3, float y3, float depth3, cv::Mat& image) const {

    // create edges for the triangle
    Edge edges[3] = {
        Edge((int)x1, (int)y1, depth1, (int)x2, (int)y2, depth2),
        Edge((int)x2, (int)y2, depth2, (int)x3, (int)y3, depth3),
        Edge((int)x3, (int)y3, depth3, (int)x1, (int)y1, depth1)
    };

    int maxLength = 0;
    int longEdge = 0;

    // find edge with the greatest length in the y axis
    for(int i = 0; i < 3; i++) {
//        std::cout << "Edge: " << edges[i].X1 << ", " << edges[i].Y1 << " - " << edges[i].X2 << ", " << edges[i].Y2 << std::endl;

        int length = edges[i].Y2 - edges[i].Y1;
        if(length > maxLength) {
            maxLength = length;
            longEdge = i;
        }
    }

    int shortEdge1 = (longEdge + 1) % 3;
    int shortEdge2 = (longEdge + 2) % 3;

    // draw spans between edges; the long edge can be drawn
    // with the shorter edges to draw the full triangle
    drawSpansBetweenEdges(edges[longEdge], edges[shortEdge1], image);
    drawSpansBetweenEdges(edges[longEdge], edges[shortEdge2], image);
}

void DepthCamera::drawSpansBetweenEdges(const Edge& e1, const Edge& e2, cv::Mat& image) const {
    // calculate difference between the y coordinates
    // of the first edge and return if 0
    float e1ydiff = (float)(e1.Y2 - e1.Y1);
    if(e1ydiff == 0.0f)
        return;

    // calculate difference between the y coordinates
    // of the second edge and return if 0
    float e2ydiff = (float)(e2.Y2 - e2.Y1);
    if(e2ydiff == 0.0f)
        return;

    // calculate differences between the x coordinates
    // and colors of the points of the edges
    float e1xdiff = (float)(e1.X2 - e1.X1);
    float e2xdiff = (float)(e2.X2 - e2.X1);
    float e1colordiff = (e1.Color2 - e1.Color1);
    float e2colordiff = (e2.Color2 - e2.Color1);

    // calculate factors to use for interpolation
    // with the edges and the step values to increase
    // them by after drawing each span
    float factor1 = (float)(e2.Y1 - e1.Y1) / e1ydiff;
    float factorStep1 = 1.0f / e1ydiff;
    float factor2 = 0.0f;
    float factorStep2 = 1.0f / e2ydiff;


    if (e2.Y1 < 0) {
        factor1 += -e2.Y1 * factorStep1;
        factor2 += -e2.Y1 * factorStep2;
    }

    // loop through the lines between the edges and draw spans
    for(int y = std::max(0, e2.Y1); y < std::min(e2.Y2, image.rows); y++) {
        if (y >= 0 && y < image.rows) {
            //cout << y << endl;
            // create and draw span
            Span span(e1.Color1 + (e1colordiff * factor1),
                      e1.X1 + (int)(e1xdiff * factor1),
                      e2.Color1 + (e2colordiff * factor2),
                      e2.X1 + (int)(e2xdiff * factor2));
            drawSpan(span, y, image);
        }

        // increase factors
        factor1 += factorStep1;
        factor2 += factorStep2;
    }
}

void DepthCamera::drawSpan(const Span &span, int y, cv::Mat& image) const {

//    std::cout << "    Span: " << span.X1 << " - " << span.X2 << std::endl;

    int xdiff = span.X2 - span.X1;
    if(xdiff == 0)
        return;

    float colordiff = span.Color2 - span.Color1;

    float factor = 0.0f;
    float factorStep = 1.0f / (float)xdiff;

    if (span.X1 < 0) {
        factor += -span.X1 * factorStep;
    }

    // draw each pixel in the span
    for(int x = std::max(0, span.X1); x < std::min(span.X2, image.cols); x++) {
        if (x >= 0 && x < image.cols) {
            float depth = span.Color1 + (colordiff * factor);
            float old_depth = image.at<float>(y, x);
            if (old_depth == 0 || old_depth > depth) {
                image.at<float>(y, x) = depth;
            }

//            std::cout << "        Pixel: " << x << " , " << y << ": " << depth << std::endl;
        }
        factor += factorStep;
    }
}

}
