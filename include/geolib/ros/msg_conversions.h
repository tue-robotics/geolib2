#ifndef GEOLIB_ROS_MSG_CONVERSIONS_H_
#define GEOLIB_ROS_MSG_CONVERSIONS_H_

#include "geolib/datatypes.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>

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
