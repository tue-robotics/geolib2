#ifndef GEOLIB_DEPTHCAMERA_H_
#define GEOLIB_DEPTHCAMERA_H_

#include <opencv2/core/core.hpp>

#include "geolib/Ray.h"
#include "geolib/math_types.h"

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include <vector>

namespace geo {

/**
 * @brief PointerMap maps pixels in a depth image to an identifier
 */
typedef std::vector<std::vector<void*> > PointerMap;

/**
 * @brief TriangleMap maps pixels in a depth image to an index in the list of triangles in the mesh.
 * Note: check with the corresponding pointermap to find which mesh is referred to!
 */
typedef std::vector<std::vector<int> > TriangleMap;

struct RasterizeResult {
    int min_x, min_y;
    int max_x, max_y;
};

static PointerMap EMPTY_POINTER_MAP;
static TriangleMap EMPTY_TRIANGLE_MAP;

class Mesh;
class DepthCamera;

/**
 * Contains instructions on how to render a depth image
 */
class RenderOptions {

    friend class DepthCamera;

public:

    RenderOptions() : back_face_culling_(true) {}

    void setMesh(const geo::Mesh& mesh) { mesh_ = &mesh; }

    /**
     * @brief setMesh: set mesh to be rendered
     * @param mesh: mesh describing the shape to be rendered
     * @param pose: pose of the origin of the mesh with respect to the virtual camera
     */
    void setMesh(const geo::Mesh& mesh, const Pose3D& pose) {
        mesh_ = &mesh;
        pose_ = pose;
    }

    void setBackFaceCulling(bool b) { back_face_culling_ = b; }

protected:

    /**
     * @brief mesh_ mesh to be rendered
     */
    const geo::Mesh* mesh_;

    /**
     * @brief pose_ pose of the mesh with respect to the virtual camera
     */
    Pose3D pose_;

    /**
     * @brief back_face_culling_ flag to optimise rendering mesh triangles facing away from the camera.
     */
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
        : geo::RenderResult(image.cols, image.rows), image_(image), pointer_(pointer), pointer_map_(pointer_map), triangle_map_(triangle_map)
    {

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

/**
 * Model of a depth camera which may be used to either
 * convert points in an image to points in 3D space
 * simulate a depth camera and render shapes in the image
 *
 * Frame conventions:
 * the frame of the camera is defined with the z-axis pointing into the camera,
 * the x-axis matches the x -axis of the image, and the y-axis matches the
 * negative y-axis of the image.
 */
class DepthCamera {

public:

    DepthCamera();

    DepthCamera(const image_geometry::PinholeCameraModel& cam_model);

    DepthCamera(const sensor_msgs::CameraInfo& cam_info);

    virtual ~DepthCamera();

    /**
     * @brief Set camera parameters from pinhole camera model
     * @param cam_model pinhole camera model
     */
    void initFromCamModel(const image_geometry::PinholeCameraModel& cam_model);

    void render(const RenderOptions& opt, RenderResult& res) const;

    /**
     * @brief rasterize: render a 3D shape onto a 2D image
     * @param shape: 3D shape to be rendered
     * @param pose: pose of the shape with respect to the camera
     * @param image: image to render the result to
     * @param pointer_map: pointer map to store an identifier of the shape
     * @param pointer: identifier of the shape
     * @param triangle_map: triangle map to store the index of a triangle in the mesh
     * @return
     */
    RasterizeResult rasterize(const Shape& shape, const Pose3D& pose, cv::Mat& image,
                              PointerMap& pointer_map = EMPTY_POINTER_MAP,
                              void* pointer = 0, TriangleMap& triangle_map = EMPTY_TRIANGLE_MAP) const;

    RasterizeResult rasterize(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, cv::Mat& image,
                              PointerMap& pointer_map = EMPTY_POINTER_MAP,
                              void* pointer = 0, TriangleMap& triangle_map = EMPTY_TRIANGLE_MAP) const;

    inline cv::Point2d project3Dto2D(const geo::Vec3d& p, int width = 0, int height = 0) const {
        return cv::Point2d((fx_ * p.x + tx_) / -p.z + cx_, (fy_ * -p.y + ty_) / -p.z + cy_);
    }

    inline double project2Dto3DX(int x) const {
        if (!cache_valid_) updateCache();
        return (x - cx_plus_tx_) / fx_;
    }

    inline double project2Dto3DY(int y) const {
        if (!cache_valid_) updateCache();
        return -(y - cy_plus_ty_) / fy_;
    }

    /**
     * convert points in an image to points in 3D space
     * @param x: x index of the 2d point in the image
     * @param y: y index of the 2d point in the image
     * @returns: (semi) unit vector indicating the direction of the beam corresponding to the pixel.
     */
    inline geo::Vec3d project2Dto3D(int x, int y) const {
        return geo::Vec3d(project2Dto3DX(x), project2Dto3DY(y), -1.0);
    }

    inline void setFocalLengths(double fx, double fy) {
        fx_ = fx;
        fy_ = fy;
    }

    inline void setOpticalCenter(double cx, double cy) {
        cx_ = cx;
        cy_ = cy;
        updateCache();
    }

    inline void setOpticalTranslation(double tx, double ty) {
        tx_ = tx;
        ty_ = ty;
        updateCache();
    }

    inline double getFocalLengthX() const { return fx_; }

    inline double getFocalLengthY() const { return fy_; }

    inline double getOpticalCenterX() const { return cx_; }

    inline double getOpticalCenterY() const { return cy_; }

    inline double getOpticalTranslationX() const { return tx_; }

    inline double getOpticalTranslationY() const { return ty_; }

protected:

    // focal length of the camera
    double fx_, fy_;

    // optical center of the camera
    double cx_, cy_;

    // optical translation of the camera
    double tx_, ty_;

    // sums stored for optimisation
    mutable double cx_plus_tx_;
    mutable double cy_plus_ty_;
    mutable bool cache_valid_;

    inline void updateCache() const
    {
        cx_plus_tx_ = cx_ + tx_;
        cy_plus_ty_ = cy_ + ty_;
        cache_valid_ = true;
    }

    void drawTriangle(const geo::Vec3d& p1, const geo::Vec3d& p2, const geo::Vec3d& p3,
                      const RenderOptions& opt, RenderResult& res, uint i_triangle) const;

    void drawTriangle2D(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3,
                        const RenderOptions& opt, RenderResult& res, uint i_triangle) const;

    void drawTrianglePart(int y_start, int y_end,
                          float x_start, float x_start_delta, float x_end, float x_end_delta,
                          float d_start, float d_start_delta, float d_end, float d_end_delta,
                          const RenderOptions& opt, RenderResult& res, uint i_triangle) const;

    void sort(const geo::Vec3d& p1, const geo::Vec3d& p2, const geo::Vec3d& p3, int dim,
              Vec3d& p_min, geo::Vec3d& p_mid, geo::Vec3d& p_max) const;

};

}

#endif
