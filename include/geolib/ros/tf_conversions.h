#ifndef GEOLIB_ROS_TF_CONVERSIONS_H_
#define GEOLIB_ROS_TF_CONVERSIONS_H_

#include "geolib/datatypes.h"

#include <tf/transform_datatypes.h>

namespace geo {

// ------------------------------ TO TF ------------------------------

void convert(const geo::Vector3& v, tf::Vector3& tf) {
    tf.setValue(v.x, v.y, v.z);
}

void convert(const geo::Quaternion& q, tf::Quaternion& tf) {
    tf.setValue(q.x, q.y, q.z, q.w);
}

void convert(const geo::Matrix3& m, tf::Matrix3x3& tf) {
    tf.setValue(m.xx, m.xy, m.xz, m.yx, m.yy, m.yz, m.zx, m.zy, m.zz);
}

void convert(const geo::Transform& t, tf::Transform& tf) {
    tf::Vector3 v_tf;
    convert(t.getOrigin(), v_tf);
    tf.setOrigin(v_tf);

    tf::Matrix3x3 m_tf;
    convert(t.getBasis(), m_tf);
    tf.setBasis(m_tf);
}

// ------------------------------ FROM TF ------------------------------

void convert(const tf::Vector3& tf, geo::Vector3& v) {
    v.x = tf.getX(); v.y = tf.getY(); v.z = tf.getZ();
}

void convert(const tf::Quaternion& tf ,geo::Quaternion& q) {
    q.x = tf.getX(); q.y = tf.getY(); q.z = tf.getZ(); q.w = tf.getW();
}

void convert(const tf::Matrix3x3& tf, geo::Matrix3& m) {
    m = geo::Matrix3(tf[0][0], tf[0][1], tf[0][2],
                     tf[1][0], tf[1][1], tf[1][2],
                     tf[2][0], tf[2][1], tf[2][2]);
}

void convert(const tf::Transform& tf, geo::Transform& t) {
    geo::Vector3 v;
    convert(tf.getOrigin(), v);
    t.setOrigin(v);

    geo::Matrix3 m;
    convert(tf.getBasis(), m);
    t.setBasis(m);
}

}

#endif
