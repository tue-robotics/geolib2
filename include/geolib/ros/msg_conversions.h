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

/**
 * @brief converting geo::Vector3 to geometry_msgs::Point message
 * @param v geo::Vector3 as input
 * @param msg filled geometery_msgs::Point message as output
 */
inline void convert(const geo::Vector3& v, geometry_msgs::Point& msg) {
    msg.x = v.x; msg.y = v.y; msg.z = v.z;
}

/**
 * @brief converting geo::Vector3 to geometry_msgs::Point32 message
 * @param v geo::Vector3 as input
 * @param msg filled geometery_msgs::Point32 message as output
 */
inline void convert(const geo::Vector3& v, geometry_msgs::Point32& msg) {
    msg.x = v.x; msg.y = v.y; msg.z = v.z;
}

/**
 * @brief converting geo::Vector3 to geometry_msgs::Vector3 message
 * @param v geo::Vector3 as input
 * @param msg filled geometery_msgs::Vector3 message as output
 */
inline void convert(const geo::Vector3& v, geometry_msgs::Vector3& msg) {
    msg.x = v.x; msg.y = v.y; msg.z = v.z;
}

/**
 * @brief converting geo::Quaternion to geometry_msgs::Quaternion
 * @param q geo::Quaternion as input
 * @param msg filled geometery_msgs::Quaternion message as output
 */
inline void convert(const geo::Quaternion& q, geometry_msgs::Quaternion& msg) {
    msg.x = q.x; msg.y = q.y; msg.z = q.z; msg.w = q.w;
}

/**
 * @brief converting geo::Matrix3 to geometry_msgs::Quaternion
 * @param r geo::Matrix3 as input
 * @param msg filled geometry_msgs::Quaternion message as output
 */
inline void convert(const geo::Matrix3& r, geometry_msgs::Quaternion& msg) {
    Quaternion q;
    r.getRotation(q);
    convert(q, msg);
}

/**
 * @brief converting geo::Transform to geometry_msgs::Pose
 * @param t geo::Transform as input
 * @param msg filled geometry_msgs::Pose as output
 */
inline void convert(const geo::Transform& t, geometry_msgs::Pose& msg) {
    convert(t.getOrigin(), msg.position);
    convert(t.getBasis(), msg.orientation);
}

/**
 * @brief converting geo::Transform to geometry_msgs::Transform
 * @param t geo::Transform as input
 * @param msg filled geometry_msgs::Transform as output
 */
inline void convert(const geo::Transform& t, geometry_msgs::Transform& msg) {
    convert(t.getOrigin(), msg.translation);
    convert(t.getBasis(), msg.rotation);
}

/**
 * @brief converting geo::TraingleI (Indices of a triangle in a vector of points) to shape_msgs::MeshTriangle
 * @param t geo::TriangleI as input
 * @param msg filled shape_msgs::MeshTriangle message as output
 */
inline void convert(const geo::TriangleI& t, shape_msgs::MeshTriangle& msg) {
    msg.vertex_indices[0] = t.i1_;
    msg.vertex_indices[1] = t.i2_;
    msg.vertex_indices[2] = t.i3_;
}

/**
 * @brief converts a geo::Mesh to shape_msgs::Mesh
 * Both use a very similar structure based on points and triangles.
 * @param m geo::Mesh as input
 * @param msg filled shape_msgs::Mesh message as output
 */
void convert(const geo::Mesh& m, shape_msgs::Mesh& msg);

// ------------------------------ FROM ROS ------------------------------

/**
 * @brief converting geometry_msgs::Point to geo::Vector3
 * @param msg geometry_msgs::Point as input
 * @param v geo::Vector3 as output
 */
inline void convert(const geometry_msgs::Point& msg, geo::Vector3& v) {
   v.x = msg.x; v.y = msg.y; v.z = msg.z;
}

/**
 * @brief converting geometry_msgs::Point32 to geo::Vector3
 * @param msg geometry_msgs::Point as input
 * @param v geo::Vector3 as output
 */
inline void convert(const geometry_msgs::Point32& msg, geo::Vector3& v) {
   v.x = msg.x; v.y = msg.y; v.z = msg.z;
}

/**
 * @brief converting geometry_msgs::Vector3 to geo::Vector3
 * @param msg geometry_msgs::Vector3 as input
 * @param v geo::Vector3 as output
 */
inline void convert(const geometry_msgs::Vector3& msg, geo::Vector3& v) {
   v.x = msg.x; v.y = msg.y; v.z = msg.z;
}

/**
 * @brief converting geometry_msgs::Quaternion to geo::Quaternion
 * @param msg geometry_msgs::Quaternion as input
 * @param q geo::Quaternion as output
 */
inline void convert(const geometry_msgs::Quaternion& msg, geo::Quaternion& q) {
    q.x = msg.x; q.y = msg.y; q.z = msg.z; q.w = msg.w;
}

/**
 * @brief converting geometry_msgs::Quaternion to geo::Matrix3
 * @param msg geometry_msgs::Quaternion as input
 * @param r geo::Matrix3 as output
 */
inline void convert(const geometry_msgs::Quaternion& msg, geo::Matrix3& r) {
    Quaternion q;
    convert(msg, q);
    r.setRotation(q);
}

/**
 * @brief converting geometry_msgs::Pose to geo::Transform
 * @param msg geometry_msgs::Pose as input
 * @param t geo::Transform as output
 */
inline void convert(const geometry_msgs::Pose& msg, geo::Transform& t) {
    convert(msg.orientation, t.R);
    convert(msg.position, t.t);
}

/**
 * @brief converting geometry_msgs::Transform to geo::Transform
 * @param msg geometry_msgs::Transform as input
 * @param t geo::Transform as output
 */
inline void convert(const geometry_msgs::Transform& msg, geo::Transform& t) {
    convert(msg.rotation, t.R);
    convert(msg.translation, t.t);
}

}

#endif
