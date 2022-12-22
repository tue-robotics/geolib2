#include "geolib/sensors/DepthCamera.h"
#include "geolib/Shape.h"

#include <image_geometry/pinhole_camera_model.h>

#include <array>
#include <vector>

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

DepthCamera::DepthCamera()
{
}

DepthCamera::DepthCamera(uint width, uint height, double fx, double fy, double cx, double cy, double tx, double ty)
{
    sensor_msgs::CameraInfo cam_info;
    cam_info.D.resize(5, 0);
    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    cam_info.K[0] = fx;
    cam_info.K[1] = 0;
    cam_info.K[2] = cx;
    cam_info.K[3] = 0;
    cam_info.K[4] = fy;
    cam_info.K[5] = cy;
    cam_info.K[6] = 0;
    cam_info.K[7] = 0;
    cam_info.K[8] = 1;

    // Rectification matrix (stereo cameras only)
    cam_info.R[0] = 1;
    cam_info.R[1] = 0;
    cam_info.R[2] = 0;
    cam_info.R[3] = 0;
    cam_info.R[4] = 1;
    cam_info.R[5] = 0;
    cam_info.R[6] = 0;
    cam_info.R[7] = 0;
    cam_info.R[8] = 1;

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    cam_info.P[0] = fx;
    cam_info.P[1] = 0;
    cam_info.P[2] = cx;
    cam_info.P[3] = tx;
    cam_info.P[4] = 0;
    cam_info.P[5] = fy;
    cam_info.P[6] = cy;
    cam_info.P[7] = ty;
    cam_info.P[8] = 0;
    cam_info.P[9] = 0;
    cam_info.P[10] = 1;
    cam_info.P[11] = 0;

    cam_info.width = width;
    cam_info.height = height;
    cam_model_.fromCameraInfo(cam_info);
}

DepthCamera::DepthCamera(const image_geometry::PinholeCameraModel& cam_model)
{
    initFromCamModel(cam_model);
}

DepthCamera::DepthCamera(const sensor_msgs::CameraInfo& cam_info)
{
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info);
    initFromCamModel(cam_model);
}

DepthCamera::~DepthCamera() {
}

void DepthCamera::initFromCamModel(const image_geometry::PinholeCameraModel& cam_model)
{
    cam_model_ = cam_model;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        RASTERIZATION
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

RasterizeResult DepthCamera::rasterize(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, cv::Mat& image,
                                       PointerMap& pointer_map, void* pointer, TriangleMap& triangle_map) const {
    return rasterize(shape, cam_pose.inverse() * obj_pose, image, pointer_map, pointer, triangle_map);
}

// -------------------------------------------------------------------------------

RasterizeResult DepthCamera::rasterize(const Shape& shape, const Pose3D& pose, cv::Mat& image,
                                       PointerMap& pointer_map, void* pointer, TriangleMap& triangle_map) const {

    RenderOptions opt;
    opt.setMesh(shape.getMesh(), pose);

    DefaultRenderResult res(image, pointer, pointer_map, triangle_map);

    render(opt, res);

    return RasterizeResult();
}

// -------------------------------------------------------------------------------

void DepthCamera::render(const RenderOptions& opt, RenderResult& res) const {
    const Mesh& mesh = *opt.mesh_;
    const Pose3D& pose = opt.pose_;

    if (mesh.getMaxRadius() < pose.getOrigin().z) {
        return;
    }

    const std::vector<geo::TriangleI>& triangles = mesh.getTriangleIs();
    const std::vector<geo::Vec3d>& points = mesh.getPoints();

    // transform points
    std::vector<geo::Vec3d> points_t(points.size());
    std::vector<bool> points_t_in_view(points.size());
    std::vector<cv::Point2f> points_2d(points.size());

    // Do not render in case no point of the mesh is inside the field of view
    bool in_view = false;
    for (unsigned int i = 0; i < points.size(); ++i) {
        points_t[i] = pose * points[i];
        points_t_in_view[i] = (points_t[i].z < near_clip_z_);
        points_2d[i] = project3Dto2D<double, float>(points_t[i]);
        if (!in_view && points_t_in_view[i])
        {
            in_view = true;
        }
    }
    if (!in_view)
        return;

    uint i_triangle = 0;
    for(std::vector<TriangleI>::const_iterator it_tri = triangles.cbegin(); it_tri != triangles.cend(); ++it_tri) {

        const geo::Vec3d& p1_3d = points_t[it_tri->i1_];
        const geo::Vec3d& p2_3d = points_t[it_tri->i2_];
        const geo::Vec3d& p3_3d = points_t[it_tri->i3_];

        uchar n_verts_in = 0;
        bool v1_in = false;
        bool v2_in = false;
        // bool v3_in = false; // Not used, because of logic can be concluded this would be true or false
        std::array<const geo::Vec3d*, 3> vIn;

        if (points_t_in_view[it_tri->i1_]) {
            ++n_verts_in;
            v1_in = true;
        }

        if (points_t_in_view[it_tri->i2_]) {
            ++n_verts_in;
            v2_in = true;
        }

        if (points_t_in_view[it_tri->i3_]) {
            ++n_verts_in;
            // v3_in = true; // Not used, because of logic can be concluded this would be true or false
        }

        if (n_verts_in == 1) {
            if (v1_in) { vIn[0] = &(p1_3d); vIn[1] = &(p2_3d); vIn[2] = &(p3_3d); }
            else if (v2_in) { vIn[0] = &(p2_3d); vIn[1] = &(p3_3d); vIn[2] = &(p1_3d); }
            else { vIn[0] = &(p3_3d); vIn[1] = &(p1_3d); vIn[2] = &(p2_3d); } // if (v3_in)

            //Parametric line stuff
            // p = v0 + v01*t
            geo::Vec3d v01 = *vIn[1] - *vIn[0];

            float t1 = ((near_clip_z_ - vIn[0]->z) / v01.z);

            geo::Vec3d new2(vIn[0]->x + v01.x * t1, vIn[0]->y + v01.y * t1, near_clip_z_);

            // Second vert point
            geo::Vec3d v02 = *vIn[2] - *vIn[0];

            float t2 = ((near_clip_z_ - vIn[0]->z) / v02.z);

            geo::Vec3d new3(vIn[0]->x + v02.x * t2, vIn[0]->y + v02.y * t2, near_clip_z_);

            drawTriangle<double, float>(*vIn[0], new2, new3, opt, res, i_triangle);
        } else if (n_verts_in == 2) {
            if (!v1_in) { vIn[0]=&(p2_3d); vIn[1]=&(p3_3d); vIn[2]=&(p1_3d); }
            else if (!v2_in) { vIn[0]=&(p3_3d); vIn[1]=&(p1_3d); vIn[2]=&(p2_3d); }
            else { vIn[0]=&(p1_3d); vIn[1]=&(p2_3d); vIn[2]=&(p3_3d); } // if (!v3_in)

            //Parametric line stuff
            // p = v0 + v01*t
            geo::Vec3d v01 = *vIn[2] - *vIn[0];

            float t1 = ((near_clip_z_ - vIn[0]->z)/v01.z);

            geo::Vec3d new2(vIn[0]->x + v01.x * t1,vIn[0]->y + v01.y * t1, near_clip_z_);

            // Second point
            geo::Vec3d v02 = *vIn[2] - *vIn[1];

            float t2 = ((near_clip_z_ - vIn[1]->z)/v02.z);

            geo::Vec3d new3(vIn[1]->x + v02.x * t2, vIn[1]->y + v02.y * t2, near_clip_z_);

            drawTriangle<double, float>(*vIn[0], *vIn[1], new2, opt, res, i_triangle);

            drawTriangle<double, float>(new2, *vIn[1], new3, opt, res, i_triangle);

        } else if (n_verts_in == 3) {
            const cv::Point2f& p1_2d = points_2d[it_tri->i1_];
            const cv::Point2f& p2_2d = points_2d[it_tri->i2_];
            const cv::Point2f& p3_2d = points_2d[it_tri->i3_];

            drawTriangle2D<float>(geo::Vec3f(p1_2d.x, p1_2d.y, 1.0f / -p1_3d.z),
                                  geo::Vec3f(p2_2d.x, p2_2d.y, 1.0f / -p2_3d.z),
                                  geo::Vec3f(p3_2d.x, p3_2d.y, 1.0f / -p3_3d.z),
                                  opt, res, i_triangle);
        }

        if (res.stop_) {
            return;
        }

        ++i_triangle;
    }
}

// -------------------------------------------------------------------------------

template<typename Tin, typename Tout>
void DepthCamera::drawTriangle(const geo::Vec3T<Tin>& p1_3d, const geo::Vec3T<Tin>& p2_3d, const geo::Vec3T<Tin>& p3_3d,
                               const RenderOptions& opt, RenderResult& res, uint i_triangle) const {
    cv::Point_<Tout> p1_2d = project3Dto2D<Tin, Tout>(p1_3d);
    cv::Point_<Tout> p2_2d = project3Dto2D<Tin, Tout>(p2_3d);
    cv::Point_<Tout> p3_2d = project3Dto2D<Tin, Tout>(p3_3d);

    drawTriangle2D<Tout>(geo::Vec3T<Tout>(p1_2d.x, p1_2d.y, 1.0f / -p1_3d.z),
                         geo::Vec3T<Tout>(p2_2d.x, p2_2d.y, 1.0f / -p2_3d.z),
                         geo::Vec3T<Tout>(p3_2d.x, p3_2d.y, 1.0f / -p3_3d.z),
                         opt, res, i_triangle);
}

// -------------------------------------------------------------------------------

template<typename T>
void DepthCamera::drawTriangle2D(const geo::Vec3T<T>& p1, const geo::Vec3T<T>& p2, const geo::Vec3T<T>& p3,
                               const RenderOptions& opt, RenderResult& res, uint i_triangle) const {

    if (!opt.back_face_culling_ || (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y) < 0) {

        int min_y = std::min<int>(p1.y, std::min<int>(p2.y, p3.y));
        int max_y = std::max<int>(p1.y, std::max<int>(p2.y, p3.y));
        int min_x = std::min<int>(p1.x, std::min<int>(p2.x, p3.x));
        int max_x = std::max<int>(p1.x, std::max<int>(p2.x, p3.x));

        if (min_x < res.getWidth() && max_x > 0 && min_y < res.getHeight() && max_y > 0) {

            const geo::Vec3T<T>* p_min=&p1;
            const geo::Vec3T<T>* p_mid=&p2;
            const geo::Vec3T<T>* p_max=&p3;
            if (min_y == max_y) {
                sort(p_min, p_mid, p_max, 0);

                drawTrianglePart(p_min->y, p_mid->y,
                     p_min->x, 0, p_max->x, 0,
                     p_min->z, 0, p_max->z, 0,
                     opt, res, i_triangle);
            } else {
                sort(p_min, p_mid, p_max, 1);

                int y_min_mid = static_cast<int>(p_mid->y) - static_cast<int>(p_min->y);
                int y_mid_max = static_cast<int>(p_max->y) - static_cast<int>(p_mid->y);
                int y_min_max = static_cast<int>(p_max->y) - static_cast<int>(p_min->y);

                geo::Vec3T<T> p_prime = (y_mid_max * *p_min + y_min_mid * *p_max) / y_min_max;

                const geo::Vec3T<T>* p_a = &p_prime;
                const geo::Vec3T<T>* p_b = p_mid;
                if (p_prime.x > p_mid->x)
                    std::swap(p_a, p_b);

                drawTrianglePart(p_min->y, p_mid->y,
                     p_min->x, (p_a->x - p_min->x) / y_min_mid, p_min->x, (p_b->x - p_min->x) / y_min_mid,
                     p_min->z, (p_a->z - p_min->z) / y_min_mid, p_min->z, (p_b->z - p_min->z) / y_min_mid,
                     opt, res, i_triangle);

                drawTrianglePart(p_mid->y, p_max->y,
                     p_a->x, (p_max->x - p_a->x) / y_mid_max, p_b->x, (p_max->x - p_b->x) / y_mid_max,
                     p_a->z, (p_max->z - p_a->z) / y_mid_max, p_b->z, (p_max->z - p_b->z) / y_mid_max,
                     opt, res, i_triangle);

            }
        }
    }
}

// -------------------------------------------------------------------------------

void DepthCamera::drawTrianglePart(int y_start, int y_end,
                                   float x_start, float x_start_delta, float x_end, float x_end_delta,
                                   float d_start, float d_start_delta, float d_end, float d_end_delta,
                                   const RenderOptions& /*opt*/, RenderResult& res, uint i_triangle) const {

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

template<typename T>
void DepthCamera::sort(const geo::Vec3T<T>*& p_min, const geo::Vec3T<T>*& p_mid, const geo::Vec3T<T>*& p_max, uchar i) const
{
    if (p_min->m[i] > p_max->m[i])
       std::swap(p_min, p_max);

    if (p_min->m[i] > p_mid->m[i])
       std::swap(p_min, p_mid);

    if (p_mid->m[i] > p_max->m[i])
       std::swap(p_mid, p_max);
}

}
