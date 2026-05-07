#include "geolib/ros/msg_conversions.h"

namespace geo {

void convert(const geo::Mesh& m, shape_msgs::msg::Mesh& msg) {
    const std::vector<Vector3>& points = m.getPoints();
    const std::vector<TriangleI>& triangles = m.getTriangleIs();

    for (std::vector<Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        geometry_msgs::msg::Point point;
        convert(*it, point);
        msg.vertices.push_back(point);
    }

    for (std::vector<TriangleI>::const_iterator it = triangles.begin(); it != triangles.end(); ++it)
    {
        shape_msgs::msg::MeshTriangle meshtriangle;
        convert(*it, meshtriangle);
        msg.triangles.push_back(meshtriangle);
    }
}

void convert(const geo::DepthCamera& cam_model, sensor_msgs::msg::CameraInfo& msg)
{
    // Distortion model and parameters
    msg.distortion_model = "plumb_bob";
    msg.d.resize(5, 0);

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    msg.k[0] = cam_model.getFocalLengthX();
    msg.k[1] = 0;
    msg.k[2] = cam_model.getOpticalCenterX();
    msg.k[3] = 0;
    msg.k[4] = cam_model.getFocalLengthY();
    msg.k[5] = cam_model.getOpticalCenterY();
    msg.k[6] = 0;
    msg.k[7] = 0;
    msg.k[8] = 1;

    // Rectification matrix (stereo cameras only)
    msg.r[0] = 1;
    msg.r[1] = 0;
    msg.r[2] = 0;
    msg.r[3] = 0;
    msg.r[4] = 1;
    msg.r[5] = 0;
    msg.r[6] = 0;
    msg.r[7] = 0;
    msg.r[8] = 1;

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    msg.p[0] = cam_model.getFocalLengthX();
    msg.p[1] = 0;
    msg.p[2] = cam_model.getOpticalCenterX();
    msg.p[3] = cam_model.getOpticalTranslationX();
    msg.p[4] = 0;
    msg.p[5] = cam_model.getFocalLengthY();
    msg.p[6] = cam_model.getOpticalCenterY();
    msg.p[7] = cam_model.getOpticalTranslationY();
    msg.p[8] = 0;
    msg.p[9] = 0;
    msg.p[10] = 1;
    msg.p[11] = 0;

    // TODO: add width and height field to DepthCamera
    msg.width = cam_model.width();
    msg.height = cam_model.height();

    msg.binning_x = 1;
    msg.binning_y = 1;
}

} // end geo namespace
