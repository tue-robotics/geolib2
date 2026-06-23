#include <geolib/math_types.h>
#include <geolib/ros/tf2_conversions.h>
#include <iostream>

#if __has_include(<tf2/LinearMath/Matrix3x3.hpp>)
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#else
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#endif

#include <stdlib.h> /* srand, rand */
#include <time.h> /* time */

// ----------------------------------------------------------------------------------------------------

double random(double min, double max)
{
    return ((double)rand() / RAND_MAX) * (max - min) + min;
}

// ----------------------------------------------------------------------------------------------------

tf2::Transform randomTransform()
{
    tf2::Matrix3x3 b;
    b.setEulerYPR(random(0, 6.283), random(0, 6.283), random(0, 6.283));
    return tf2::Transform(b, tf2::Vector3(random(-10, 10), random(-10, 10), random(-10, 10)));
}

// ----------------------------------------------------------------------------------------------------

void print(const tf2::Matrix3x3& m)
{
    std::cout << "[ " << m[0][0] << " " << m[0][1] << " " << m[0][2] << std::endl;
    std::cout << "  " << m[1][0] << " " << m[1][1] << " " << m[1][2] << std::endl;
    std::cout << "  " << m[2][0] << " " << m[2][1] << " " << m[2][2] << " ]" << std::endl;
}
void print(const tf2::Vector3& v)
{
    std::cout << "[ " << v[0] << " " << v[1] << " " << v[2] << " ]" << std::endl;
}

void print(const tf2::Transform& t)
{
    print(t.getBasis());
    print(t.getOrigin());
}

// ----------------------------------------------------------------------------------------------------

bool equals(double a, double b)
{
    return std::abs(a - b) < 1e-10;
}

bool equals(const geo::Transform3& T, const tf2::Transform& T_tf)
{
    if (!equals(T.t.x, T_tf.getOrigin().getX()) || !equals(T.t.y, T_tf.getOrigin().getY()) ||
        !equals(T.t.z, T_tf.getOrigin().getZ()))
    {
        return false;
    }

    geo::Quaternion q = T.getQuaternion();
    tf2::Quaternion q_tf = T_tf.getRotation();

    if (!equals(q.x, q_tf.getX()) || !equals(q.y, q_tf.getY()) || !equals(q.z, q_tf.getZ()) ||
        !equals(q.w, q_tf.getW()))
    {
        return false;
    }

    return true;
}

bool equals(const geo::Transform3& T, const tf2::Transform& T_tf, const std::string& msg)
{
    if (!equals(T, T_tf))
    {
        std::cout << "ERROR\t" << msg << std::endl;
        std::cout << "TF: " << std::endl;
        print(T_tf);
        std::cout << std::endl;
        std::cout << "GEOLIB:" << std::endl << T << std::endl;
        return false;
    }
    else
    {
        std::cout << "OK\t" << msg << std::endl;
        return true;
    }
}

// ----------------------------------------------------------------------------------------------------

int main()
{
    // initialize random seed
    srand(time(NULL));

    tf2::Transform t1_tf = randomTransform();
    tf2::Transform t2_tf = randomTransform();

    geo::Transform3 t1, t2;
    geo::convert(t1_tf, t1);
    geo::convert(t2_tf, t2);

    if (!equals(t1, t1_tf, "equals(Transform, Transform)") ||
        !equals(t1 * t2, t1_tf * t2_tf, "Transform * Transform") ||
        !equals(t1.inverse() * t2, t1_tf.inverse() * t2_tf, "Transform.inverse() * Transform") ||
        !equals(t1.inverseTimes(t2), t1_tf.inverseTimes(t2_tf), "Transform.inverseTimes(Transform)") ||
        !equals(geo::Transform3::identity() * t1, t1_tf, "Transform.identity() * Transform"))
    {
        std::cout << "ERROR" << std::endl;
        return -1;
    }

    std::cout << "All OK" << std::endl;

    return 0;
}
