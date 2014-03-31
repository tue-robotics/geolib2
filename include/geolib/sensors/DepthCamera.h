#ifndef GEOLIB_DEPTHCAMERA_H_
#define GEOLIB_DEPTHCAMERA_H_

#include <opencv2/core/core.hpp>

#include "geolib/Ray.h"
#include "geolib/math_types.h"

namespace geo {

typedef std::vector<std::vector<void*> > PointerMap;
typedef std::vector<std::vector<int> > TriangleMap;

struct RasterizeResult {
    int min_x, min_y;
    int max_x, max_y;
};

static PointerMap EMPTY_POINTER_MAP;
static TriangleMap EMPTY_TRIANGLE_MAP;

class RenderOptions {

    RenderOptions() : pointer_(0) {}

protected:

    void* pointer_;

};

class RenderOutput {

    const cv::Mat& getDepthImage() const { return image_; }
    const PointerMap& getPointerMap() const { return pointer_map_; }
    const TriangleMap& getTriangleMap() const { return triangle_map_; }

protected:

    cv::Mat image_;
    PointerMap pointer_map_;
    TriangleMap triangle_map_;

};


class DepthCamera {

public:

    DepthCamera();

    virtual ~DepthCamera();

    RasterizeResult rasterize(const Shape& shape, const Pose3D& pose, cv::Mat& image,
                              PointerMap& pointer_map = EMPTY_POINTER_MAP,
                              void* pointer = 0, TriangleMap& triangle_map = EMPTY_TRIANGLE_MAP) const;

    RasterizeResult rasterize(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, cv::Mat& image,
                              PointerMap& pointer_map = EMPTY_POINTER_MAP,
                              void* pointer = 0, TriangleMap& triangle_map = EMPTY_TRIANGLE_MAP) const;

    inline cv::Point2d project3Dto2D(const Vector3 p, int width = 0, int height = 0) const {
        return cv::Point2d((fx_ * p.x() + tx_) / -p.z() + cx_, (fy_ * -p.y() + ty_) / -p.z() + cy_);
    }

    inline double project2Dto3DX(int x) const {
        return (x - cx_plus_tx_) / fx_;
    }

    inline double project2Dto3DY(int y) const {
        return -(y - cy_plus_ty_) / fy_;
    }

    inline Vector3 project2Dto3D(int x, int y) const {
        return Vector3(project2Dto3DX(x), project2Dto3DY(y), -1.0);
    }

    inline void setFocalLengths(double fx, double fy) {
        fx_ = fx;
        fy_ = fy;
    }

    inline void setOpticalCenter(double cx, double cy) {
        cx_ = cx;
        cy_ = cy;
        cx_plus_tx_ = cx_ + tx_;
        cy_plus_ty_ = cy_ + ty_;
    }

    inline void setOpticalTranslation(double tx, double ty) {
        tx_ = tx;
        ty_ = ty;
        cx_plus_tx_ = cx_ + tx_;
        cy_plus_ty_ = cy_ + ty_;
    }

    inline double getFocalLengthX() const { return fx_; }

    inline double getFocalLengthY() const { return fy_; }

    inline double getOpticalCenterX() const { return cx_; }

    inline double getOpticalCenterY() const { return cy_; }

    inline double getOpticalTranslationX() const { return tx_; }

    inline double getOpticalTranslationY() const { return ty_; }

protected:

    double fx_, fy_;
    double cx_, cy_;
    double tx_, ty_;
    double cx_plus_tx_;
    double cy_plus_ty_;

    void drawTriangle(const Vector3& p1, const Vector3& p2, const Vector3& p3, cv::Mat& image,
                      PointerMap& pointer_map, void* pointer, TriangleMap& triangle_map, int i_triangle, RasterizeResult& res) const;

    void drawTriangle2D(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3,
                        cv::Mat& image, PointerMap& pointer_map, void* pointer,
                        TriangleMap& triangle_map, int i_triangle, RasterizeResult& res) const;

    void drawTrianglePart(cv::Mat& depth_image, int y_start, int y_end,
                          float x_start, float x_start_delta, float x_end, float x_end_delta,
                          float d_start, float d_start_delta, float d_end, float d_end_delta,
                          PointerMap& pointer_map, void* pointer,
                          TriangleMap& triangle_map, int i_triangle) const;

    void sort(const geo::Vec3f& p1, const geo::Vec3f& p2, const geo::Vec3f& p3, int dim,
              Vec3f& p_min,geo::Vec3f& p_mid, geo::Vec3f& p_max) const;

};

}

#endif
