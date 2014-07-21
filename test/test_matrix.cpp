#include <geolib/math_types.h>
#include <geolib/ros/tf_conversions.h>
#include <iostream>

#include <profiling/Timer.h>

//#define USE_TF

#include <tf/transform_datatypes.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

// ----------------------------------------------------------------------------------------------------

double random(double min, double max) {
    return ((double)rand() / RAND_MAX) * (max - min) + min;
}

// ----------------------------------------------------------------------------------------------------

tf::Transform randomTransform() {
    tf::Matrix3x3 b;
    b.setEulerYPR(random(0, 6.283), random(0, 6.283), random(0, 6.283));
    return tf::Transform(b, tf::Vector3(random(-10, 10), random(-10, 10), random(-10, 10)));
}

// ----------------------------------------------------------------------------------------------------

void print(const tf::Matrix3x3& m) {
    std::cout << "[ " << m[0][0] << " " << m[0][1] << " " << m[0][2] << std::endl;
    std::cout << "  " << m[1][0] << " " << m[1][1] << " " << m[1][2] << std::endl;
    std::cout << "  " << m[2][0] << " " << m[2][1] << " " << m[2][2] << " ]" << std::endl;
}
void print(const tf::Vector3& v) {
    std::cout << "[ " << v[0] << " " << v[1] << " " << v[2] << " ]" << std::endl;
}

void print(const tf::Transform& t) {
    print(t.getBasis());
    print(t.getOrigin());
}

// ----------------------------------------------------------------------------------------------------

bool equals(double a, double b) {
    return std::abs(a - b) < 1e-10;
}

bool equals(const geo::Transform3& T, const tf::Transform& T_tf) {
    if (    !equals(T.t.x, T_tf.getOrigin().getX()) ||
            !equals(T.t.y, T_tf.getOrigin().getY()) ||
            !equals(T.t.z, T_tf.getOrigin().getZ())) {
        return false;
    }

    geo::Quaternion q = T.getQuaternion();
    tf::Quaternion q_tf = T_tf.getRotation();

    if (    !equals(q.x, q_tf.getX()) ||
            !equals(q.y, q_tf.getY()) ||
            !equals(q.z, q_tf.getZ()) ||
            !equals(q.w, q_tf.getW())) {
        return false;
    }

    return true;
}

bool equals(const geo::Transform3& T, const tf::Transform& T_tf, const std::string& msg) {
    if (!equals(T, T_tf)) {
        std::cout << "ERROR\t" << msg << std::endl;
        std::cout << "TF: " << std::endl;
        print(T_tf);
        std::cout << std::endl;
        std::cout << "GEOLIB:" << std::endl << T << std::endl;
        return false;
    } else {
        std::cout << "OK\t" << msg << std::endl;
        return true;
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv) {

    // initialize random seed
    srand (time(NULL));

    tf::Transform t1_tf = randomTransform();
    tf::Transform t2_tf = randomTransform();

    geo::Transform3 t1, t2;
    geo::convert(t1_tf, t1);
    geo::convert(t2_tf, t2);

    if (    !equals(t1, t1_tf, "equals(Transform, Transform)") ||
            !equals(t1 * t2, t1_tf * t2_tf, "Transform * Transform") ||
            !equals(t1.inverse() * t2, t1_tf.inverse() * t2_tf, "Transform.inverse() * Transform") ||
            !equals(t1.inverseTimes(t2), t1_tf.inverseTimes(t2_tf), "Transform.inverseTimes(Transform)") ||
            !equals(geo::Transform3::identity() * t1, t1_tf, "Transform.identity() * Transform")) {

        std::cout << "ERROR" << std::endl;
        return -1;
    }

    std::cout << "All OK" << std::endl;

    return 0;
}
