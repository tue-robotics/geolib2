#ifndef GEOLIB_MATRIX_H_
#define GEOLIB_MATRIX_H_

#include <ostream>
#include <cstring>
#include <cmath>

namespace geo {

typedef double real;

class Vector3 {

    friend class Matrix3x3;
    friend class Transform;

public:

    Vector3() {}

    Vector3(real x, real y, real z) {
        v_[0] = x;
        v_[1] = y;
        v_[2] = z;
    }

    Vector3(const real* v) {
        memcpy(v_, v, 3 * sizeof(real));
    }

    inline const real& x() const { return v_[0]; }
    inline const real& y() const { return v_[1]; }
    inline const real& z() const { return v_[2]; }

    inline const real& getX() const { return v_[0]; }
    inline const real& getY() const { return v_[1]; }
    inline const real& getZ() const { return v_[2]; }

    inline void setX(real x) { v_[0] = x; }
    inline void setY(real y) { v_[1] = y; }
    inline void setZ(real z) { v_[2] = z; }

    inline real length() const {
        return sqrt(v_[0]*v_[0] + v_[1]*v_[1] + v_[2]*v_[2]);
    }

    inline real length2() const {
        return v_[0]*v_[0] + v_[1]*v_[1] + v_[2]*v_[2];
    }

    inline void normalize() {
        real l = length();
        v_[0] /= l; v_[1] /= l; v_[2] /= l;
    }

    inline real dot(const Vector3& v) const {
        return v_[0]*v.v_[0] + v_[1]*v.v_[1] + v_[2]*v.v_[2];
    }

    inline Vector3 operator+(const Vector3& v) const {
        return Vector3(v_[0]+v.v_[0], v_[1]+v.v_[1], v_[2]+v.v_[2]);
    }

    inline Vector3 operator-(const Vector3& v) const {
        return Vector3(v_[0]-v.v_[0], v_[1]-v.v_[1], v_[2]-v.v_[2]);
    }

    inline Vector3 operator*(real f) const {
        return Vector3(v_[0] * f, v_[1] * f, v_[2] * f);
    }

    inline Vector3 operator/(real f) const {
        return Vector3(v_[0] / f, v_[1] / f, v_[2] / f);
    }

    friend std::ostream& operator<< (std::ostream& out, const Vector3& v) {
        out << "[ " << v.v_[0] << " " << v.v_[1] << " " << v.v_[2] << " ]";
        return out;
    }

    friend Vector3 operator*(real s, const Vector3& v) {
        return v * s;
    }

    friend Vector3 operator-(const Vector3& v) {
        return Vector3(-v.v_[0], -v.v_[1], -v.v_[2]);
    }

protected:

    real v_[3];
};



class Matrix3x3 {

    friend class Transform;

public:

    Matrix3x3() {}

    Matrix3x3(real m11, real m12, real m13,
              real m21, real m22, real m23,
              real m31, real m32, real m33) //:
//        m_{m11, m12, m13, m21, m22, m23, m31, m32, m33}
    {
        m_[0]= m11;
        m_[1]= m12;
        m_[2]= m13;
        m_[3]= m21;
        m_[4]= m22;
        m_[5]= m23;
        m_[6]= m31;
        m_[7]= m32;
        m_[8]= m33;
    }

    Matrix3x3(const real* m) {
        memcpy(m_, m, 9 * sizeof(real));
    }

    void setRPY(real roll, real pitch, real yaw)  {
        real ci = cos(roll);
        real cj = cos(pitch);
        real ch = cos(yaw);
        real si = sin(roll);
        real sj = sin(pitch);
        real sh = sin(yaw);
        real cc = ci * ch;
        real cs = ci * sh;
        real sc = si * ch;
        real ss = si * sh;

        m_[0] = cj * ch; m_[1] = sj * sc - cs; m_[2] = sj * cc + ss;
        m_[3] = cj * sh; m_[4] = sj * ss + cc, m_[5] = sj * cs - sc;
        m_[6] = -sj;     m_[7] = cj * si;      m_[8] = cj * ci ;
    }

    inline Vector3 getRow(int i) {
        return Vector3(m_[i*3], m_[i*3+1], m_[i*3+2]);
    }

    Matrix3x3 operator*(const Matrix3x3& n) const {
        return Matrix3x3(m_[0]*n.m_[0]+m_[1]*n.m_[3]+m_[2]*n.m_[6], m_[0]*n.m_[1]+m_[1]*n.m_[4]+m_[2]*n.m_[7], m_[0]*n.m_[2]+m_[1]*n.m_[5]+m_[2]*n.m_[8],
                         m_[3]*n.m_[0]+m_[4]*n.m_[3]+m_[5]*n.m_[6], m_[3]*n.m_[1]+m_[4]*n.m_[4]+m_[5]*n.m_[7], m_[3]*n.m_[2]+m_[4]*n.m_[5]+m_[5]*n.m_[8],
                         m_[6]*n.m_[0]+m_[7]*n.m_[3]+m_[8]*n.m_[6], m_[6]*n.m_[1]+m_[7]*n.m_[4]+m_[8]*n.m_[7], m_[6]*n.m_[2]+m_[7]*n.m_[5]+m_[8]*n.m_[8]);
    }

    Vector3 operator*(const Vector3& v) const {
        return Vector3(m_[0]*v.v_[0]+m_[1]*v.v_[1]+m_[2]*v.v_[2],
                       m_[3]*v.v_[0]+m_[4]*v.v_[1]+m_[5]*v.v_[2],
                       m_[6]*v.v_[0]+m_[7]*v.v_[1]+m_[8]*v.v_[2]);
    }

    friend std::ostream& operator<< (std::ostream& out, const Matrix3x3& m) {
        out << "[ " << m.m_[0] << " " << m.m_[1] << " " << m.m_[2] << std::endl
                    << m.m_[3] << " " << m.m_[4] << " " << m.m_[5] << std::endl
                    << m.m_[6] << " " << m.m_[7] << " " << m.m_[8] << " ]";
        return out;
    }

protected:

    real m_[9];

};

class Transform {

public:

    Transform() {}

    Transform(real x, real y, real z, real roll = 0, real pitch = 0, real yaw = 0) {
        o_[0] = x; o_[1] = y; o_[2] = z;
        setRPY(roll, pitch, yaw);
    }

    Transform(real m11, real m12, real m13,
              real m21, real m22, real m23,
              real m31, real m32, real m33,
              real x, real y, real z) {

        r_[0] = m11; r_[1] = m12; r_[2] = m13;
        r_[3] = m21; r_[4] = m22; r_[5] = m23;
        r_[6] = m31; r_[7] = m32; r_[8] = m33;
        o_[0] = x; o_[1] = y; o_[2] = z;
    }

    Transform(const Matrix3x3& r, const Vector3& v) {
        memcpy(o_, v.v_, 3 * sizeof(real));
        memcpy(r_, r.m_, 9 * sizeof(real));
    }

    inline Vector3 operator*(const Vector3& v) const {
        return Vector3(r_[0]*v.v_[0]+r_[1]*v.v_[1]+r_[2]*v.v_[2]+o_[0],
                       r_[3]*v.v_[0]+r_[4]*v.v_[1]+r_[5]*v.v_[2]+o_[1],
                       r_[6]*v.v_[0]+r_[7]*v.v_[1]+r_[8]*v.v_[2]+o_[2]);
    }

    inline Transform operator*(const Transform& t) const {
        return Transform(r_[0]*t.r_[0]+r_[1]*t.r_[3]+r_[2]*t.r_[6], r_[0]*t.r_[1]+r_[1]*t.r_[4]+r_[2]*t.r_[7], r_[0]*t.r_[2]+r_[1]*t.r_[5]+r_[2]*t.r_[8],
                         r_[3]*t.r_[0]+r_[4]*t.r_[3]+r_[5]*t.r_[6], r_[3]*t.r_[1]+r_[4]*t.r_[4]+r_[5]*t.r_[7], r_[3]*t.r_[2]+r_[4]*t.r_[5]+r_[5]*t.r_[8],
                         r_[6]*t.r_[0]+r_[7]*t.r_[3]+r_[8]*t.r_[6], r_[6]*t.r_[1]+r_[7]*t.r_[4]+r_[8]*t.r_[7], r_[6]*t.r_[2]+r_[7]*t.r_[5]+r_[8]*t.r_[8],

                         r_[0]*t.o_[0]+r_[1]*t.o_[1]+r_[2]*t.o_[2]+o_[0],
                         r_[3]*t.o_[0]+r_[4]*t.o_[1]+r_[5]*t.o_[2]+o_[1],
                         r_[6]*t.o_[0]+r_[7]*t.o_[1]+r_[8]*t.o_[2]+o_[2]);
    }

    inline Vector3 getOrigin() const {
        return Vector3(o_);
    }

    inline Matrix3x3 getBasis() const {
        return Matrix3x3(r_);
    }

    inline Transform inverse() const {
        return Transform(r_[0], r_[3], r_[6],
                         r_[1], r_[4], r_[7],
                         r_[2], r_[5], r_[8],

                         -r_[0]*o_[0]-r_[3]*o_[1]-r_[6]*o_[2],
                         -r_[1]*o_[0]-r_[4]*o_[1]-r_[7]*o_[2],
                         -r_[2]*o_[0]-r_[5]*o_[1]-r_[8]*o_[2]);
    }

    void setRPY(real roll, real pitch, real yaw)  {
        real ci = cos(roll);
        real cj = cos(pitch);
        real ch = cos(yaw);
        real si = sin(roll);
        real sj = sin(pitch);
        real sh = sin(yaw);
        real cc = ci * ch;
        real cs = ci * sh;
        real sc = si * ch;
        real ss = si * sh;

        r_[0] = cj * ch; r_[1] = sj * sc - cs; r_[2] = sj * cc + ss;
        r_[3] = cj * sh; r_[4] = sj * ss + cc, r_[5] = sj * cs - sc;
        r_[6] = -sj;     r_[7] = cj * si;      r_[8] = cj * ci ;
    }

    friend std::ostream& operator<< (std::ostream& out, const Transform& t) {
        out << "[ " << t.r_[0] << " " << t.r_[1] << " " << t.r_[2] << "   [ " << t.o_[0] << " ]" << std::endl
                    << t.r_[3] << " " << t.r_[4] << " " << t.r_[5] << "   [ " << t.o_[1] << " ]" << std::endl
                    << t.r_[6] << " " << t.r_[7] << " " << t.r_[8] << " ] [ " << t.o_[2] << " ]";
        return out;
    }

protected:

    real o_[3];
    real r_[9];

};

}

#endif
