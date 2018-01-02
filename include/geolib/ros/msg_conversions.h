#ifndef GEOLIB_ROS_MSG_CONVERSIONS_H_
#define GEOLIB_ROS_MSG_CONVERSIONS_H_

#include "geolib/datatypes.h"
#include <geolib/Mesh.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>
#include <shape_msgs/Mesh.h>

namespace geo {

// ------------------------------ TO ROS ------------------------------

inline void convert(const geo::Vector3& v, geometry_msgs::Point& msg) {
    msg.x = v.x; msg.y = v.y; msg.z = v.z;
}

inline void convert(const geo::Vector3& v, geometry_msgs::Point32& msg) {
    msg.x = v.x; msg.y = v.y; msg.z = v.z;
}

inline void convert(const geo::Vector3& v, geometry_msgs::Vector3& msg) {
    msg.x = v.x; msg.y = v.y; msg.z = v.z;
}

inline void convert(const geo::Quaternion& q, geometry_msgs::Quaternion& msg) {
    msg.x = q.x; msg.y = q.y; msg.z = q.z; msg.w = q.w;
}

inline void convert(const geo::Matrix3& r, geometry_msgs::Quaternion& msg) {
    Quaternion q;
    r.getRotation(q);
    convert(q, msg);
}

inline void convert(const geo::Transform& t, geometry_msgs::Pose& msg) {
    convert(t.getOrigin(), msg.position);
    convert(t.getBasis(), msg.orientation);
}

inline void convert(const geo::Transform& t, geometry_msgs::Transform& msg) {
    convert(t.getOrigin(), msg.translation);
    convert(t.getBasis(), msg.rotation);
}

inline void convert(const TriangleI& t, shape_msgs::MeshTriangle& msg) {
    msg.vertex_indices[0] = t.i1_;
    msg.vertex_indices[1] = t.i2_;
    msg.vertex_indices[2] = t.i3_;
}

void convert(const geo::Mesh& m, shape_msgs::Mesh& msg) {
    std::vector<Vector3> points = m.getPoints();
    std::vector<TriangleI> triangles = m.getTriangleIs();

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

// ------------------------------ FROM ROS ------------------------------

inline void convert(const geometry_msgs::Point& msg, geo::Vector3& v) {
   v.x = msg.x; v.y = msg.y; v.z = msg.z;
}

inline void convert(const geometry_msgs::Point32& msg, geo::Vector3& v) {
   v.x = msg.x; v.y = msg.y; v.z = msg.z;
}

inline void convert(const geometry_msgs::Vector3& msg, geo::Vector3& v) {
   v.x = msg.x; v.y = msg.y; v.z = msg.z;
}

inline void convert(const geometry_msgs::Quaternion& msg, geo::Quaternion& q) {
    q.x = msg.x; q.y = msg.y; q.z = msg.z; q.w = msg.w;
}

inline void convert(const geometry_msgs::Quaternion& msg, geo::Matrix3& r) {
    Quaternion q;
    convert(msg, q);
    r.setRotation(q);
}

inline void convert(const geometry_msgs::Pose& msg, geo::Transform& t) {
    convert(msg.orientation, t.R);
    convert(msg.position, t.t);
}

inline void convert(const geometry_msgs::Transform& msg, geo::Transform& t) {
    convert(msg.rotation, t.R);
    convert(msg.translation, t.t);
}

}

#endif
