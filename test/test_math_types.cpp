#include <gtest/gtest.h>

#include <geolib/math_types.h>

// These are characterization tests for a behaviour-preserving refactor:
// they pin the public access surface (named members, operator[], operator())
// and must pass both before and after the unions are removed.

TEST(MathTypes, Vec2IndexAliasesNamedMembers)
{
    geo::Vec2T<double> v(1.0, 2.0);
    EXPECT_EQ(v[0], v.x);
    EXPECT_EQ(v[1], v.y);
    EXPECT_DOUBLE_EQ(v[0], 1.0);
    EXPECT_DOUBLE_EQ(v[1], 2.0);

    v[0] = 7.0;
    EXPECT_DOUBLE_EQ(v.x, 7.0);
}

TEST(MathTypes, Vec2PointerConstructor)
{
    const double values[2] = {3.0, 4.0};
    geo::Vec2T<double> v(values);
    EXPECT_DOUBLE_EQ(v.x, 3.0);
    EXPECT_DOUBLE_EQ(v.y, 4.0);
}

TEST(MathTypes, Vec3IndexAliasesNamedMembers)
{
    geo::Vec3T<double> v(1.0, 2.0, 3.0);
    EXPECT_EQ(v[0], v.x);
    EXPECT_EQ(v[1], v.y);
    EXPECT_EQ(v[2], v.z);
    EXPECT_DOUBLE_EQ(v[2], 3.0);

    v[2] = 9.0;
    EXPECT_DOUBLE_EQ(v.z, 9.0);
}

TEST(MathTypes, Vec3PointerConstructor)
{
    const double values[3] = {3.0, 4.0, 5.0};
    geo::Vec3T<double> v(values);
    EXPECT_DOUBLE_EQ(v.x, 3.0);
    EXPECT_DOUBLE_EQ(v.y, 4.0);
    EXPECT_DOUBLE_EQ(v.z, 5.0);
}

TEST(MathTypes, Mat2IndexAndCallAliasMembers)
{
    geo::Mat2T<double> m(1.0, 2.0, 3.0, 4.0); // xx, xy, yx, yy
    EXPECT_EQ(m[0], m.xx);
    EXPECT_EQ(m[1], m.xy);
    EXPECT_EQ(m[2], m.yx);
    EXPECT_EQ(m[3], m.yy);
    // operator() row-major: (i,j) -> i*2+j
    EXPECT_EQ(m(0, 0), m.xx);
    EXPECT_EQ(m(0, 1), m.xy);
    EXPECT_EQ(m(1, 0), m.yx);
    EXPECT_EQ(m(1, 1), m.yy);
}

TEST(MathTypes, QuaternionIndexAliasesNamedMembers)
{
    geo::QuaternionT<double> q(1.0, 2.0, 3.0, 4.0); // x, y, z, w
    EXPECT_EQ(q[0], q.x);
    EXPECT_EQ(q[1], q.y);
    EXPECT_EQ(q[2], q.z);
    EXPECT_EQ(q[3], q.w);

    q[3] = 8.0;
    EXPECT_DOUBLE_EQ(q.w, 8.0);
}

TEST(MathTypes, Mat3IndexAndCallAliasMembers)
{
    geo::Mat3T<double> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    EXPECT_EQ(m[0], m.xx);
    EXPECT_EQ(m[1], m.xy);
    EXPECT_EQ(m[2], m.xz);
    EXPECT_EQ(m[3], m.yx);
    EXPECT_EQ(m[4], m.yy);
    EXPECT_EQ(m[5], m.yz);
    EXPECT_EQ(m[6], m.zx);
    EXPECT_EQ(m[7], m.zy);
    EXPECT_EQ(m[8], m.zz);
    // operator() row-major: (i,j) -> i*3+j
    EXPECT_EQ(m(0, 0), m.xx);
    EXPECT_EQ(m(1, 2), m.yz);
    EXPECT_EQ(m(2, 1), m.zy);
}

TEST(MathTypes, Mat3PointerConstructor)
{
    const double values[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    geo::Mat3T<double> m(values);
    EXPECT_DOUBLE_EQ(m.xx, 1.0);
    EXPECT_DOUBLE_EQ(m.yy, 5.0);
    EXPECT_DOUBLE_EQ(m.zz, 9.0);
}

TEST(MathTypes, Mat3GetRowGetColumn)
{
    geo::Mat3T<double> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    geo::Vec3T<double> row0 = m.getRow(0);
    EXPECT_DOUBLE_EQ(row0.x, 1.0);
    EXPECT_DOUBLE_EQ(row0.y, 2.0);
    EXPECT_DOUBLE_EQ(row0.z, 3.0);
    geo::Vec3T<double> col0 = m.getColumn(0);
    EXPECT_DOUBLE_EQ(col0.x, 1.0);
    EXPECT_DOUBLE_EQ(col0.y, 4.0);
    EXPECT_DOUBLE_EQ(col0.z, 7.0);
}

TEST(MathTypes, Mat3RotationRoundTripsThroughQuaternion)
{
    // Exercises setRotation + getRotation, which use indexed access internally.
    geo::Mat3T<double> m = geo::Mat3T<double>::identity();
    geo::QuaternionT<double> q;
    m.getRotation(q);
    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);

    geo::Mat3T<double> m2;
    m2.setRotation(q);
    EXPECT_NEAR(m2.xx, 1.0, 1e-12);
    EXPECT_NEAR(m2.yy, 1.0, 1e-12);
    EXPECT_NEAR(m2.zz, 1.0, 1e-12);
    EXPECT_NEAR(m2.xy, 0.0, 1e-12);
}
