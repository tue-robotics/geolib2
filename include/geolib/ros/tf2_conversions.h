#ifndef GEOLIB_ROS_TF2_CONVERSIONS_H_
#define GEOLIB_ROS_TF2_CONVERSIONS_H_

#include "geolib/datatypes.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace geo {

// ------------------------------ TO TF2 -----------------------------

inline void convert(const geo::Vector3& v, tf2::Vector3& tf) {
    tf.setValue(v.x, v.y, v.z);
}

inline void convert(const geo::Quaternion& q, tf2::Quaternion& tf) {
    tf.setValue(q.x, q.y, q.z, q.w);
}

inline void convert(const geo::Matrix3& m, tf2::Matrix3x3& tf) {
    tf.setValue(m.xx, m.xy, m.xz, m.yx, m.yy, m.yz, m.zx, m.zy, m.zz);
}

inline void convert(const geo::Transform& t, tf2::Transform& tf) {
    convert(t.getOrigin(), tf.getOrigin());
    convert(t.getBasis(), tf.getBasis());
}

// ------------------------------ FROM TF2 -----------------------------

inline void convert(const tf2::Vector3& tf, geo::Vector3& v) {
    v.x = tf.getX(); v.y = tf.getY(); v.z = tf.getZ();
}

inline void convert(const tf2::Quaternion& tf ,geo::Quaternion& q) {
    q.x = tf.getX(); q.y = tf.getY(); q.z = tf.getZ(); q.w = tf.getW();
}

inline void convert(const tf2::Matrix3x3& tf, geo::Matrix3& m) {
    m = geo::Matrix3(tf[0][0], tf[0][1], tf[0][2],
                     tf[1][0], tf[1][1], tf[1][2],
                     tf[2][0], tf[2][1], tf[2][2]);
}

inline void convert(const tf2::Transform& tf, geo::Transform& t) {
    convert(tf.getOrigin(), t.t);
    convert(tf.getBasis(), t.R);
}

}

#endif
