#include <geolib/matrix.h>
#include <tf/transform_datatypes.h>

#include <iostream>

#include <profiling/Timer.h>

int main(int argc, char **argv) {

    geo::Matrix3x3 m1(1, 2, 3, 4, 5, 6, 7, 8, 9);
    geo::Matrix3x3 m2(10, 11, 12, 13, 14, 15, 16, 17, 18);
    geo::Vector3 v(1, 2, 3);

    geo::Matrix3x3 m3;
    geo::Vector3 v2;

    std::vector<geo::Matrix3x3> result(10000000);

    Timer t;
    t.start();
    for(unsigned int i = 0; i < 10000000; ++i) {
        result[i] = m1 * m2;
    }
    t.stop();
    std::cout << t.getElapsedTimeInMilliSec() << " ms" << std::endl;

//    std::cout << result[0] << std::endl;
//    std::cout << m3[0][0] << " " << m3[0][1] << " " << m3[0][2] << std::endl;
//    std::cout << m3[1][0] << " " << m3[1][1] << " " << m3[1][2] << std::endl;
//    std::cout << m3[2][0] << " " << m3[2][1] << " " << m3[2][2] << std::endl;

    return 0;
}
