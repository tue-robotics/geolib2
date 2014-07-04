#include "geolib/sensors/DepthCamera.h"
#include "geolib/Shape.h"

namespace geo {

void DefaultRenderResult::renderPixel(int x, int y, float depth, int i_triangle) {
    float old_depth = image_.at<float>(y, x);
    if (old_depth == 0 || old_depth > depth) {
        image_.at<float>(y, x) = depth;

        if (pointer_) {
            pointer_map_[x][y] = pointer_;
        }

        if (!triangle_map_.empty()) {
            triangle_map_[x][y] = i_triangle;
        }
    }
}

DepthCamera::DepthCamera() {
}

DepthCamera::~DepthCamera() {

}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        RASTERIZATION
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

RasterizeResult DepthCamera::rasterize(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, cv::Mat& image,
                                       PointerMap& pointer_map, void* pointer, TriangleMap& triangle_map) const {
    return rasterize(shape, cam_pose.inverse() * obj_pose, image, pointer_map, pointer);
}

// -------------------------------------------------------------------------------

RasterizeResult DepthCamera::rasterize(const Shape& shape, const Pose3D& pose, cv::Mat& image,
                                       PointerMap& pointer_map, void* pointer, TriangleMap& triangle_map) const {

    RenderOptions opt;
    opt.setMesh(shape.getMesh(), pose);

    DefaultRenderResult res(image, pointer, pointer_map, triangle_map);

    render(opt, res);

    RasterizeResult res2;

    image = res.image_;
    pointer_map = res.pointer_map_;
    triangle_map = res.triangle_map_;

    return res2;
}

// -------------------------------------------------------------------------------

void DepthCamera::render(const RenderOptions& opt, RenderResult& res) const {
    const Mesh& mesh = *opt.mesh_;
    const Pose3D& pose = opt.pose_;

    Transform pose_in = pose;

    if (mesh.getMaxRadius() < pose_in.getOrigin().z) {
        return;
    }

    double near_clip_z = -0.1;

    const std::vector<TriangleI>& triangles = mesh.getTriangleIs();
    const std::vector<Vector3>& points = mesh.getPoints();

    // transform points
    std::vector<Vector3> points_t(points.size());
    std::vector<cv::Point2d> points_proj(points.size());

    for(unsigned int i = 0; i < points.size(); ++i) {
        points_t[i] = pose_in * points[i];
        points_proj[i] = project3Dto2D(points_t[i]);
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

        if (p1_3d.z < near_clip_z) {
            ++n_verts_in;
            v1_in = true;
        }

        if (p2_3d.z < near_clip_z) {
            ++n_verts_in;
            v2_in = true;
        }

        if (p3_3d.z < near_clip_z) {
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

            float t1 = ((near_clip_z - (*vIn[0]).z) / v01.z );

            Vector3 new2(vIn[0]->x + v01.x * t1, vIn[0]->y + v01.y * t1, near_clip_z);

            // Second vert point
            Vector3 v02 = *vIn[2] - *vIn[0];

            float t2 = ((near_clip_z - (*vIn[0]).z) / v02.z);

            Vector3 new3(vIn[0]->x + v02.x * t2, vIn[0]->y + v02.y * t2, near_clip_z);

            drawTriangle(*vIn[0], new2, new3, opt, res, i_triangle);
        } else if (n_verts_in == 2) {
            if (!v1_in) { vIn[0]=&(p2_3d); vIn[1]=&(p3_3d); vIn[2]=&(p1_3d); }
            if (!v2_in) { vIn[0]=&(p3_3d); vIn[1]=&(p1_3d); vIn[2]=&(p2_3d); }
            if (!v3_in) { vIn[0]=&(p1_3d); vIn[1]=&(p2_3d); vIn[2]=&(p3_3d); }

            //Parametric line stuff
            // p = v0 + v01*t
            Vector3 v01 = *vIn[2] - *vIn[0];

            float t1 = ((near_clip_z - (*vIn[0]).z)/v01.z );

            Vector3 new2((*vIn[0]).x + v01.x * t1,(*vIn[0]).y + v01.y * t1, near_clip_z);

            // Second point
            Vector3 v02 = *vIn[2] - *vIn[1];

            float t2 = ((near_clip_z - (*vIn[1]).z)/v02.z);

            Vector3 new3((*vIn[1]).x + v02.x * t2, (*vIn[1]).y + v02.y * t2, near_clip_z);

            drawTriangle(*vIn[0], *vIn[1], new2, opt, res, i_triangle);

            drawTriangle(new2, *vIn[1], new3, opt, res, i_triangle);

        } else if (n_verts_in == 3) {
            const cv::Point2d& p1_2d = points_proj[it_tri->i1_];
            const cv::Point2d& p2_2d = points_proj[it_tri->i2_];
            const cv::Point2d& p3_2d = points_proj[it_tri->i3_];

            drawTriangle2D(Vec3f(p1_2d.x, p1_2d.y, 1.0f / -p1_3d.z),
                           Vec3f(p2_2d.x, p2_2d.y, 1.0f / -p2_3d.z),
                           Vec3f(p3_2d.x, p3_2d.y, 1.0f / -p3_3d.z),
                           opt, res, i_triangle);
        }

        if (res.stop_) {
            return;
        }

        ++i_triangle;
    }
}

// -------------------------------------------------------------------------------

void DepthCamera::drawTriangle(const Vector3& p1_3d, const Vector3& p2_3d, const Vector3& p3_3d,
                               const RenderOptions& opt, RenderResult& res, int i_triangle) const {
    cv::Point2d p1_2d = project3Dto2D(p1_3d);
    cv::Point2d p2_2d = project3Dto2D(p2_3d);
    cv::Point2d p3_2d = project3Dto2D(p3_3d);

    drawTriangle2D(Vec3f(p1_2d.x, p1_2d.y, 1.0f / -p1_3d.z),
                   Vec3f(p2_2d.x, p2_2d.y, 1.0f / -p2_3d.z),
                   Vec3f(p3_2d.x, p3_2d.y, 1.0f / -p3_3d.z),
                   opt, res, i_triangle);
}

// -------------------------------------------------------------------------------

void DepthCamera::drawTriangle2D(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3,
                               const RenderOptions& opt, RenderResult& res, int i_triangle) const {

    if (!opt.back_face_culling_ || (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y) < 0) {

        int min_y = std::min<int>(p1.y, std::min<int>(p2.y, p3.y));
        int max_y = std::max<int>(p1.y, std::max<int>(p2.y, p3.y));
        int min_x = std::min<int>(p1.x, std::min<int>(p2.x, p3.x));
        int max_x = std::max<int>(p1.x, std::max<int>(p2.x, p3.x));

        if (min_x < res.getWidth() && max_x > 0 && min_y < res.getHeight() && max_y > 0) {

            if (min_y == max_y) {
                Vec3f p_min, p_mid, p_max;
                sort(p1, p2, p3, 0, p_min, p_mid, p_max);

                drawTrianglePart(p_min.y, p_mid.y,
                     p_min.x, 0, p_max.x, 0,
                     p_min.z, 0, p_max.z, 0,
                     opt, res, i_triangle);
            } else {
                Vec3f p_min, p_mid, p_max;
                sort(p1, p2, p3, 1, p_min, p_mid, p_max);

                int y_min_mid = (int)p_mid.y - (int)p_min.y;
                int y_mid_max = (int)p_max.y - (int)p_mid.y;
                int y_min_max = (int)p_max.y - (int)p_min.y;

                Vec3f p_prime = (y_mid_max * p_min + y_min_mid * p_max) / y_min_max;

                Vec3f p_a, p_b;
                if (p_prime.x < p_mid.x) {
                    p_a = p_prime; p_b = p_mid;
                } else {
                    p_a = p_mid; p_b = p_prime;
                }

                drawTrianglePart(p_min.y, p_mid.y,
                     p_min.x, (p_a.x - p_min.x) / y_min_mid, p_min.x, (p_b.x - p_min.x) / y_min_mid,
                     p_min.z, (p_a.z - p_min.z) / y_min_mid, p_min.z, (p_b.z - p_min.z) / y_min_mid,
                     opt, res, i_triangle);

                drawTrianglePart(p_mid.y, p_max.y,
                     p_a.x, (p_max.x - p_a.x) / y_mid_max, p_b.x, (p_max.x - p_b.x) / y_mid_max,
                     p_a.z, (p_max.z - p_a.z) / y_mid_max, p_b.z, (p_max.z - p_b.z) / y_mid_max,
                     opt, res, i_triangle);

            }
        }
    }
}

// -------------------------------------------------------------------------------

void DepthCamera::drawTrianglePart(int y_start, int y_end,
                                   float x_start, float x_start_delta, float x_end, float x_end_delta,
                                   float d_start, float d_start_delta, float d_end, float d_end_delta,
                                   const RenderOptions& opt, RenderResult& res, int i_triangle) const {

//    cv::Mat& image = res.image_;

    if (y_start < 0) {
        d_start += d_start_delta * -y_start;
        d_end += d_end_delta * -y_start;
        x_start += x_start_delta * -y_start;
        x_end += x_end_delta * -y_start;
        y_start = 0;
    }

    y_end = std::min(res.getHeight() - 1, y_end);

    for(int y = y_start; y <= y_end; ++y) {
        float d = d_start;
        float d_delta = (d_end - d_start) / (x_end - x_start);

        int x_start2;
        if (x_start < 0) {
            d += d_delta * -x_start;
            x_start2 = 0;
        } else {
            x_start2 = x_start;
        }

        int x_end2 = std::min(res.getWidth() - 1, (int)x_end);

        for(int x = x_start2; x <= x_end2; ++x) {
            float depth = 1.0f / d;

            res.renderPixel(x, y, depth, i_triangle);
            d += d_delta;
        }

        d_start+= d_start_delta;
        d_end += d_end_delta;
        x_start += x_start_delta;
        x_end += x_end_delta;
    }
}

// -------------------------------------------------------------------------------

void DepthCamera::sort(const geo::Vec3f& p1, const geo::Vec3f& p2, const geo::Vec3f& p3, int i,
                       Vec3f& p_min,geo::Vec3f& p_mid, geo::Vec3f& p_max) const {

    if (p1.m[i] < p2.m[i]) {
        if (p2.m[i] < p3.m[i]) {
            p_min = p1; p_mid = p2; p_max = p3;
        } else if (p3.m[i] < p1.m[i]) {
            p_min = p3; p_mid = p1; p_max = p2;
        } else {
            p_min = p1; p_mid = p3; p_max = p2;
        }
    } else {
        if (p1.m[i] < p3.m[i]) {
            p_min = p2; p_mid = p1; p_max = p3;
        } else if (p3.m[i] < p2.m[i]) {
            p_min = p3; p_mid = p2; p_max = p1;
        } else {
            p_min = p2; p_mid = p3; p_max = p1;
        }
    }
}

}
