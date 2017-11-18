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

class Mesh;
class DepthCamera;

class RenderOptions {

    friend class DepthCamera;

public:

    RenderOptions() : back_face_culling_(true) {}

    void setMesh(const geo::Mesh& mesh) { mesh_ = &mesh; }
    void setMesh(const geo::Mesh& mesh, const Pose3D& pose) {
        mesh_ = &mesh;
        pose_ = pose;
    }

    void setBackFaceCulling(bool b) { back_face_culling_ = b; }

protected:

    const geo::Mesh* mesh_;
    Pose3D pose_;
    bool back_face_culling_;

};

class RenderResult {

    friend class DepthCamera;

public:

    RenderResult(int width, int height) : stop_(false), width_(width), height_(height) {
    }

    virtual ~RenderResult() {}

    virtual void renderPixel(int x, int y, float depth, int i_triangle) = 0;

    void stop() { stop_ = true; }

    int getWidth() const { return width_; }

    int getHeight() const { return height_; }

private:

    bool stop_;

    int width_, height_;

};

class DefaultRenderResult : public RenderResult {

    friend class DepthCamera;

public:

    DefaultRenderResult(cv::Mat& image, void* pointer, PointerMap& pointer_map, TriangleMap& triangle_map)
        : geo::RenderResult(image.cols, image.rows), image_(image), pointer_(pointer), pointer_map_(pointer_map), triangle_map_(triangle_map) {

        // reserve pointer map
        if (pointer_) {
            if (pointer_map_.empty() || (int)pointer_map_.size() != image.cols || (int)pointer_map_[0].size() != image.rows) {
                pointer_map_.resize(image.cols, std::vector<void*>(image.rows, (void*)NULL));
            }
        }

        // reserve triangle map
        if (triangle_map_.empty() || (int)triangle_map_.size() != image.cols || (int)triangle_map_[0].size() != image.rows) {
            triangle_map_.resize(image_.cols, std::vector<int>(image_.rows, -1));
        }
    }

    virtual ~DefaultRenderResult() {}

    const cv::Mat& getDepthImage() const { return image_; }
    const PointerMap& getPointerMap() const { return pointer_map_; }
    const TriangleMap& getTriangleMap() const { return triangle_map_; }

    virtual void renderPixel(int x, int y, float depth, int i_triangle);

protected:

    cv::Mat& image_;
    void* pointer_;
    PointerMap& pointer_map_;
    TriangleMap& triangle_map_;

};


class DepthCamera {

public:

    DepthCamera();

    virtual ~DepthCamera();

    void render(const RenderOptions& opt, RenderResult& res) const;

    RasterizeResult rasterize(const Shape& shape, const Pose3D& pose, cv::Mat& image,
                              PointerMap& pointer_map = EMPTY_POINTER_MAP,
                              void* pointer = 0, TriangleMap& triangle_map = EMPTY_TRIANGLE_MAP) const;

    RasterizeResult rasterize(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, cv::Mat& image,
                              PointerMap& pointer_map = EMPTY_POINTER_MAP,
                              void* pointer = 0, TriangleMap& triangle_map = EMPTY_TRIANGLE_MAP) const;

    inline cv::Point2d project3Dto2D(const Vector3& p, int width = 0, int height = 0) const {
        return cv::Point2d((fx_ * p.x + tx_) / -p.z + cx_, (fy_ * -p.y + ty_) / -p.z + cy_);
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

    void drawTriangle(const Vector3& p1, const Vector3& p2, const Vector3& p3,
                      const RenderOptions& opt, RenderResult& res, int i_triangle) const;

    void drawTriangle2D(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3,
                        const RenderOptions& opt, RenderResult& res, int i_triangle) const;

    void drawTrianglePart(int y_start, int y_end,
                          float x_start, float x_start_delta, float x_end, float x_end_delta,
                          float d_start, float d_start_delta, float d_end, float d_end_delta,
                          const RenderOptions& opt, RenderResult& res, int i_triangle) const;

    void sort(const geo::Vec3f& p1, const geo::Vec3f& p2, const geo::Vec3f& p3, int dim,
              Vec3f& p_min,geo::Vec3f& p_mid, geo::Vec3f& p_max) const;

};

}

#endif
