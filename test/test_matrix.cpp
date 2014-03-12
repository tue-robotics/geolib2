#include <geolib/matrix.h>
#include <iostream>

#include <profiling/Timer.h>

#define USE_TF

#include <tf/transform_datatypes.h>

int main(int argc, char **argv) {

    tf::Matrix3x3 tf_rot;
    tf_rot.setEulerYPR(1.57, -2, 0.3);

#ifdef USE_TF
    tf::Matrix3x3 rot = tf_rot;
    tf::Vector3 trans(0, 1, 2);

    tf::Transform t(rot, trans);

    tf::Vector3 v(1, 2, 3);

    std::vector<tf::Vector3> result(10000000);
#else
    geo::Matrix3x3 rot(tf_rot[0][0], tf_rot[0][1], tf_rot[0][2],
                      tf_rot[1][0], tf_rot[1][1], tf_rot[1][2],
                      tf_rot[2][0], tf_rot[2][1], tf_rot[2][2]);
    geo::Vector3 trans(0, 1, 2);

    geo::Transform t(trans,rot);

    geo::Vector3 v(1, 2, 3);

    std::vector<geo::Vector3> result(10000000);
#endif

    Timer timer;
    timer.start();
    for(unsigned int i = 0; i < 10000000; ++i) {
        result[i] = t * v;
    }
    timer.stop();
    std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

#ifdef USE_TF
    std::cout << result[0][0] << " " << result[0][1] << " " << result[0][2] << std::endl;
#else
    std::cout << result[0] << std::endl;
#endif

//    std::cout << m3[0][0] << " " << m3[0][1] << " " << m3[0][2] << std::endl;
//    std::cout << m3[1][0] << " " << m3[1][1] << " " << m3[1][2] << std::endl;
//    std::cout << m3[2][0] << " " << m3[2][1] << " " << m3[2][2] << std::endl;

    return 0;
}
