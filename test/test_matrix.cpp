#include <geolib/matrix.h>
#include <iostream>

#include <profiling/Timer.h>

//#define USE_TF

#include <tf/transform_datatypes.h>

#ifdef USE_TF
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
#else
void print(const geo::Matrix3x3& m) {
    std::cout << m << std::endl;
}
void print(const geo::Vector3& v) {
    std::cout << v << std::endl;
}

void print(const geo::Transform& t) {
    std::cout << t << std::endl;
}
#endif

int main(int argc, char **argv) {

    tf::Matrix3x3 tf_rot1;
    tf_rot1.setEulerYPR(1.57, -2, 0.3);
    tf::Vector3 tf_v1(1, 2, -3);

    tf::Matrix3x3 tf_rot2;
    tf_rot2.setEulerYPR(-0.6, 4, -8.2);
    tf::Vector3 tf_v2(-0.3, 2, -1.7);

    tf::Vector3 tf_v(5, 6, 7);

#ifdef USE_TF
    tf::Transform t1(tf_rot1, tf_v1);
    tf::Transform t2(tf_rot2, tf_v2);
    tf::Vector3 v = tf_v;

    tf::Transform t3;
    std::vector<tf::Vector3> result(10000000);
#else
    geo::Matrix3x3 rot1(tf_rot1[0][0], tf_rot1[0][1], tf_rot1[0][2],
                        tf_rot1[1][0], tf_rot1[1][1], tf_rot1[1][2],
                        tf_rot1[2][0], tf_rot1[2][1], tf_rot1[2][2]);

    geo::Transform t1(rot1, geo::Vector3(tf_v1.x(), tf_v1.y(), tf_v1.z()));

    geo::Matrix3x3 rot2(tf_rot2[0][0], tf_rot2[0][1], tf_rot2[0][2],
                        tf_rot2[1][0], tf_rot2[1][1], tf_rot2[1][2],
                        tf_rot2[2][0], tf_rot2[2][1], tf_rot2[2][2]);

    geo::Transform t2(rot2, geo::Vector3(tf_v2.x(), tf_v2.y(), tf_v2.z()));

    geo::Vector3 v(tf_v.x(), tf_v.y(), tf_v.z());

    geo::Transform t3;
    std::vector<geo::Vector3> result(10000000);
#endif

    t3 = t1 * t2;

    Timer timer;
    timer.start();
    for(unsigned int i = 0; i < 10000000; ++i) {
        result[i] = t3 * v;
    }
    timer.stop();
    std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

    print(t1);
    std::cout << std::endl;
    print(t2);
    std::cout << std::endl;
    print(t3);
    std::cout << std::endl;

    return 0;
}
