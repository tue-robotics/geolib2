#include "geolib/ros/msg_conversions.h"

namespace geo {

void convert(const geo::Mesh& m, shape_msgs::Mesh& msg) {
    const std::vector<Vector3>& points = m.getPoints();
    const std::vector<TriangleI>& triangles = m.getTriangleIs();

    for (std::vector<Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        geometry_msgs::Point point;
        convert(*it, point);
        msg.vertices.push_back(point);
    }

    for (std::vector<TriangleI>::const_iterator it = triangles.begin(); it != triangles.end(); ++it)
    {
        shape_msgs::MeshTriangle meshtriangle;
        convert(*it, meshtriangle);
        msg.triangles.push_back(meshtriangle);
    }
}

void convert(const geo::DepthCamera& cam_model, sensor_msgs::CameraInfo& msg)
{
    // Distortion model and parameters
    msg.distortion_model = "plumb_bob";
    msg.D.resize(5, 0);

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    msg.K[0] = cam_model.getFocalLengthX();
    msg.K[1] = 0;
    msg.K[2] = cam_model.getOpticalCenterX();
    msg.K[3] = 0;
    msg.K[4] = cam_model.getFocalLengthY();
    msg.K[5] = cam_model.getOpticalCenterY();
    msg.K[6] = 0;
    msg.K[7] = 0;
    msg.K[8] = 1;

    // Rectification matrix (stereo cameras only)
    msg.R[0] = 1;
    msg.R[1] = 0;
    msg.R[2] = 0;
    msg.R[3] = 0;
    msg.R[4] = 1;
    msg.R[5] = 0;
    msg.R[6] = 0;
    msg.R[7] = 0;
    msg.R[8] = 1;

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    msg.P[0] = cam_model.getFocalLengthX();
    msg.P[1] = 0;
    msg.P[2] = cam_model.getOpticalCenterX();
    msg.P[3] = cam_model.getOpticalTranslationX();
    msg.P[4] = 0;
    msg.P[5] = cam_model.getFocalLengthY();
    msg.P[6] = cam_model.getOpticalCenterY();
    msg.P[7] = cam_model.getOpticalTranslationY();
    msg.P[8] = 0;
    msg.P[9] = 0;
    msg.P[10] = 1;
    msg.P[11] = 0;

    // TODO: add width and height field to DepthCamera
    msg.width = cam_model.width();
    msg.height = cam_model.height();

    msg.binning_x = 1;
    msg.binning_y = 1;
}

} // end geo namespace
