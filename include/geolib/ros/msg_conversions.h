#ifndef GEOLIB_ROS_MSG_CONVERSIONS_H_
#define GEOLIB_ROS_MSG_CONVERSIONS_H_

#include "geolib/datatypes.h"
#include <geometry_msgs/Pose.h>

namespace geo {

// ------------------------------ TO ROS ------------------------------

void convert(const geo::Vector3& v, geometry_msgs::Point& msg) {
    msg.x = v.x; msg.y = v.y; msg.z = v.z;
}

void convert(const geo::Quaternion& q, geometry_msgs::Quaternion& msg) {
    msg.x = q.x; msg.y = q.y; msg.z = q.z; msg.w = q.w;
}

void convert(const geo::Matrix3& r, geometry_msgs::Quaternion& msg) {
    Quaternion q;
    r.getRotation(q);
    convert(q, msg);
}

void convert(const geo::Transform& t, geometry_msgs::Pose& msg) {
    convert(t.getOrigin(), msg.position);
    convert(t.getBasis(), msg.orientation);
}

// ------------------------------ FROM ROS ------------------------------

void convert(const geometry_msgs::Point& msg, geo::Vector3& v) {
   v.x = msg.x; v.y = msg.y; v.z = msg.z;
}

void convert(const geometry_msgs::Quaternion& msg, geo::Quaternion& q) {
    q.x = msg.x; q.y = msg.y; q.z = msg.z; q.w = msg.w;

}

void convert(const geometry_msgs::Quaternion& msg, geo::Matrix3& r) {
    Quaternion q;
    convert(msg, q);
    r.setRotation(q);
}

void convert(const geometry_msgs::Pose& msg, geo::Transform& t) {
    Matrix3 r;
    convert(msg.orientation, r);
    t.setBasis(r);

    Vector3 v;
    convert(msg.position, v);
    t.setOrigin(v);
}

}

#endif
