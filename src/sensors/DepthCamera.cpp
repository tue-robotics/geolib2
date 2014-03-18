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
#ifdef GEOLIB_USE_TF
    tf::Transform pose_in = pose.inverse();
#else
    Transform pose_in = pose.inverse();
#endif

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

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        RASTERIZATION
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

RasterizeResult DepthCamera::rasterize(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, cv::Mat& image,
                                       PointerMap& pointer_map, void* pointer, TriangleMap& triangle_map) const {
#ifdef GEOLIB_USE_TF
    tf::Transform t = cam_pose.inverse() * obj_pose;
    return rasterize(shape, geo::Pose3D(t.getOrigin(), t.getRotation()), image, pointer_map, pointer);
#else
    return rasterize(shape, cam_pose.inverse() * obj_pose, image, pointer_map, pointer);
#endif
}

RasterizeResult DepthCamera::rasterize(const Shape& shape, const Pose3D& pose, cv::Mat& image,
                                       PointerMap& pointer_map, void* pointer, TriangleMap& triangle_map) const {

    // reserve pointer map and triangle map
    if (pointer) {
        if ((int)pointer_map.size() != image.cols || (int)pointer_map[0].size() != image.rows) {
            pointer_map.resize(image.cols, std::vector<void*>(image.rows, NULL));
        }

        if ((int)triangle_map.size() != image.cols || (int)triangle_map[0].size() != image.rows) {
            triangle_map.resize(image.cols, std::vector<int>(image.rows, -1));
        }
    }

    RasterizeResult res;
    res.min_x = image.cols;
    res.min_y = image.rows;
    res.max_x = 0;
    res.max_y = 0;

#ifdef GEOLIB_USE_TF
    tf::Transform pose_in = pose;
#else
    Transform pose_in = pose;
#endif

    if (shape.getMaxRadius() < pose_in.getOrigin().z()) {
        return res;
    }

    //pose_in.setOrigin(-pose.getOrigin());
    //tf::Transform pose_in = Pose3D(0, 0, -5, 2.3, 0.3, 0.3);//pose.inverse();

    double near_clip_z = -0.1;

    const std::vector<TriangleI>& triangles = shape.getMesh().getTriangleIs();
    const std::vector<Vector3>& points = shape.getMesh().getPoints();

    // transform points
    std::vector<Vector3> points_t(points.size());
    std::vector<cv::Point2d> points_proj(points.size());

    for(unsigned int i = 0; i < points.size(); ++i) {
        points_t[i] = pose_in * points[i];
        points_proj[i] = project3Dto2D(points_t[i], image.cols, image.rows);
    }

    int i_triangle = 0;
    for(std::vector<TriangleI>::const_iterator it_tri = triangles.begin(); it_tri != triangles.end(); ++it_tri) {
        const Vector3& p1_3d = points_t[it_tri->i1_];
        const Vector3& p2_3d = points_t[it_tri->i2_];
        const Vector3& p3_3d = points_t[it_tri->i3_];

        int n_verts_in = 0;
        bool v1_in = false;
        bool v2_in = false;
        bool v3_in = false;
        const Vector3* vIn[3];

        if (p1_3d.z() < near_clip_z) {
            ++n_verts_in;
            v1_in = true;
        }

        if (p2_3d.z() < near_clip_z) {
            ++n_verts_in;
            v2_in = true;
        }

        if (p3_3d.z() < near_clip_z) {
            ++n_verts_in;
            v3_in = true;
        }

        if (n_verts_in == 1) {
            if (v1_in) { vIn[0] = &(p1_3d); vIn[1] = &(p2_3d); vIn[2] = &(p3_3d); }
            if (v2_in) { vIn[0] = &(p2_3d); vIn[1] = &(p3_3d); vIn[2] = &(p1_3d); }
            if (v3_in) { vIn[0] = &(p3_3d); vIn[1] = &(p1_3d); vIn[2] = &(p2_3d); }

            //Parametric line stuff
            // p = v0 + v01*t
            Vector3 v01 = *vIn[1] - *vIn[0];

            float t1 = ((near_clip_z - (*vIn[0]).z()) / v01.z() );

            Vector3 new2(vIn[0]->x() + v01.x() * t1, vIn[0]->y() + v01.y() * t1, near_clip_z);

            // Second vert point
            Vector3 v02 = *vIn[2] - *vIn[0];

            float t2 = ((near_clip_z - (*vIn[0]).z()) / v02.z());

            Vector3 new3(vIn[0]->x() + v02.x() * t2, vIn[0]->y() + v02.y() * t2, near_clip_z);

            drawTriangle(*vIn[0], new2, new3, image, pointer_map, pointer, triangle_map, i_triangle, res);
        } else if (n_verts_in == 2) {
            if (!v1_in) { vIn[0]=&(p2_3d); vIn[1]=&(p3_3d); vIn[2]=&(p1_3d); }
            if (!v2_in) { vIn[0]=&(p3_3d); vIn[1]=&(p1_3d); vIn[2]=&(p2_3d); }
            if (!v3_in) { vIn[0]=&(p1_3d); vIn[1]=&(p2_3d); vIn[2]=&(p3_3d); }

            //Parametric line stuff
            // p = v0 + v01*t
            Vector3 v01 = *vIn[2] - *vIn[0];

            float t1 = ((near_clip_z - (*vIn[0]).z())/v01.z() );

            Vector3 new2((*vIn[0]).x() + v01.x() * t1,(*vIn[0]).y() + v01.y() * t1, near_clip_z);

            // Second point
            Vector3 v02 = *vIn[2] - *vIn[1];

            float t2 = ((near_clip_z - (*vIn[1]).z())/v02.z());

            Vector3 new3((*vIn[1]).x() + v02.x() * t2, (*vIn[1]).y() + v02.y() * t2, near_clip_z);

            drawTriangle(*vIn[0], *vIn[1], new2, image, pointer_map, pointer, triangle_map, i_triangle, res);

            drawTriangle(new2, *vIn[1], new3, image, pointer_map, pointer, triangle_map, i_triangle, res);

        } else if (n_verts_in == 3) {
            const cv::Point2d& p1_2d = points_proj[it_tri->i1_];
            const cv::Point2d& p2_2d = points_proj[it_tri->i2_];
            const cv::Point2d& p3_2d = points_proj[it_tri->i3_];

            drawTriangle(p1_2d.x, p1_2d.y, -p1_3d.z(),
                         p2_2d.x, p2_2d.y, -p2_3d.z(),
                         p3_2d.x, p3_2d.y, -p3_3d.z(),
                         image, pointer_map, pointer, triangle_map, i_triangle, res);
        }

        ++i_triangle;
    }

    /*
    for(int y = 0; y < image.rows; ++y) {
        for(int x = 0; x < image.cols; ++x) {
            if (image.at<float>(y, x) > 9) {
                image.at<float>(y, x) = 0.0 / 0.0;
            }
        }
    }
    */

    return res;
}

void DepthCamera::drawTriangle(const Vector3& p1_3d, const Vector3& p2_3d, const Vector3& p3_3d, cv::Mat& image,
                               PointerMap& pointer_map, void* pointer, TriangleMap& triangle_map, int i_triangle,
                               RasterizeResult& res) const {
    cv::Point2d p1_2d = project3Dto2D(p1_3d, image.cols, image.rows);
    cv::Point2d p2_2d = project3Dto2D(p2_3d, image.cols, image.rows);
    cv::Point2d p3_2d = project3Dto2D(p3_3d, image.cols, image.rows);

    drawTriangle(p1_2d.x, p1_2d.y, -p1_3d.z(),
                 p2_2d.x, p2_2d.y, -p2_3d.z(),
                 p3_2d.x, p3_2d.y, -p3_3d.z(), image, pointer_map, pointer, triangle_map, i_triangle, res);
}


void DepthCamera::drawTriangle(float x1, float y1, float depth1,
                               float x2, float y2, float depth2,
                               float x3, float y3, float depth3, cv::Mat& image,
                               PointerMap& pointer_map, void* pointer,
                               TriangleMap& triangle_map, int i_triangle,
                               RasterizeResult& res) const {

    if ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1) < 0) {
        int min_x = std::min<int>(x1, std::min<int>(x2, x3));
        int min_y = std::min<int>(y1, std::min<int>(y2, y3));
        int max_x = std::max<int>(x1, std::max<int>(x2, x3));
        int max_y = std::max<int>(y1, std::max<int>(y2, y3));

        if (min_x < image.cols && max_x > 0 && min_y < image.rows && max_y > 0) {
            res.min_x = std::max(0, std::min<int>(res.min_x, min_x));
            res.min_y = std::max(0, std::min<int>(res.min_y, min_y));
            res.max_x = std::min(image.cols - 1, std::max<int>(res.max_x, max_x));
            res.max_y = std::min(image.rows - 1, std::max<int>(res.max_y, max_y));

            float depth1_inv = 1 / depth1;
            float depth2_inv = 1 / depth2;
            float depth3_inv = 1 / depth3;

            // create edges for the triangle
            Edge edges[3] = {
                Edge((int)x1, (int)y1, depth1_inv, (int)x2, (int)y2, depth2_inv),
                Edge((int)x2, (int)y2, depth2_inv, (int)x3, (int)y3, depth3_inv),
                Edge((int)x3, (int)y3, depth3_inv, (int)x1, (int)y1, depth1_inv)
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
            drawSpansBetweenEdges(edges[longEdge], edges[shortEdge1], image, pointer_map, pointer, triangle_map, i_triangle);
            drawSpansBetweenEdges(edges[longEdge], edges[shortEdge2], image, pointer_map, pointer, triangle_map, i_triangle);
        }
    }
}

void DepthCamera::drawSpansBetweenEdges(const Edge& e1, const Edge& e2, cv::Mat& image,
                                        PointerMap& pointer_map, void* pointer,
                                        TriangleMap& triangle_map, int i_triangle) const {
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
            drawSpan(span, y, image, pointer_map, pointer, triangle_map, i_triangle);
        }

        // increase factors
        factor1 += factorStep1;
        factor2 += factorStep2;
    }
}

void DepthCamera::drawSpan(const Span &span, int y, cv::Mat& image,
                           PointerMap& pointer_map, void* pointer,
                           TriangleMap& triangle_map, int i_triangle) const {

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
            float depth = 1 / (span.Color1 + (colordiff * factor));
            float old_depth = image.at<float>(y, x);
            if (old_depth == 0 || old_depth > depth) {
                image.at<float>(y, x) = depth;
                if (pointer) {
                    pointer_map[x][y] = pointer;
                }

                if (!triangle_map.empty()) {
                    triangle_map[x][y] = i_triangle;
                }
            }

//            std::cout << "        Pixel: " << x << " , " << y << ": " << depth << std::endl;
        }
        factor += factorStep;
    }
}

}
