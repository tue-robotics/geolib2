#ifndef GEOLIB_MATH_TYPES_H_
#define GEOLIB_MATH_TYPES_H_

#include <iostream>
#include <cmath>

namespace geo {

// --------------------------------------------------------------------------------

template<typename T>
class Vec2
{

public:
  Vec2() {}
  Vec2(T x_, T y_) : x(x_), y(y_) {}
  Vec2(T value) : x(value), y(value) {}

  ~Vec2() {}

  /// returns dot product
  T dot(const Vec2& v) const { return x * v.x + y * v.y; }

  /// returns addition of this and v
  Vec2 operator+(const Vec2& v) const {  return Vec2(x + v.x, y + v.y); }

  /// returns this minus v
  Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }

  /// multiplies vector with a scalar
  Vec2 operator*(T s) const { return Vec2(x * s, y * s); }

  /// divides vector by scalar
  Vec2 operator/(T s) const { return Vec2(x / s, y / s); }

  /// multiplies vector with a scalar
  friend Vec2 operator*(T s, const Vec2& v) { return Vec2(v.x * s, v.y * s); }

  /// Returns the length of the vector
  T length() const { return sqrt(x * x + y * y); }

  /// Returns the squared length of the vector
  T length2() const { return x * x + y * y; }

  /// Returns the normalized version of the vector
  Vec2 normalized() const { T len = length(); return Vec2(x / len, y / len); }

  Vec2 &operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
  Vec2 &operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }
  Vec2 &operator*=(const Vec2& v) { x *= v.x; y *= v.y; return *this; }
  Vec2 &operator/=(const Vec2& v) { x /= v.x; y /= v.y; return *this; }

  Vec2 &operator*=(T s) { x *= s; y *= s; return *this; }
  Vec2 &operator/=(T s) { x /= s; y /= s; return *this; }

  // serialize vector to stream
  friend std::ostream& operator<< (std::ostream& out, const Vec2& v) {
      out << "[ " << v.x << " " << v.y << " ]";
      return out;
  }

  union {
      struct { T x, y; };
      T m[2];
  };
};

// --------------------------------------------------------------------------------

template<typename T>
class Vec3
{

public:
  Vec3() {}
  Vec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
  Vec3(T value) : x(value), y(value), z(value) {}

  ~Vec3() {}

  /// returns dot product
  T dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

  /// returns cross product
  T cross(const Vec3& v) const { return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x ); }

  /// returns addition with v
  Vec3 operator+(const Vec3& v) const {  return Vec3(x + v.x, y + v.y, z + v.z); }

  /// returns this minus v
  Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }

  /// multiplies vector with a scalar
  Vec3 operator*(T s) const { return Vec3(x * s, y * s, z * s); }

  /// divides vector by scalar
  Vec3 operator/(T s) const { return Vec3(x / s, y / s, z / s); }

  /// multiplies vector with a scalar
  friend Vec3 operator*(T s, const Vec3& v) { return Vec3(v.x * s, v.y * s, v.z * s); }

  /// Returns the length of the vector
  T length() const { return sqrt(x * x + y * y + z * z); }

  /// Returns the squared length of the vector
  T length2() const { return x * x + y * y + z * z; }

  /// Returns the normalized version of the vector
  Vec3 normalized() const { T len = length(); return Vec3(x / len, y / len, z / len); }

  friend Vec3 operator-(const Vec3& v) { return Vec3(-v.x, -v.y, -v.z); }

  Vec3 &operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
  Vec3 &operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
  Vec3 &operator*=(const Vec3& v) { x *= v.x; y *= v.y; z *= v.z; return *this; }
  Vec3 &operator/=(const Vec3& v) { x /= v.x; y /= v.y; z /= v.z; return *this; }

  Vec3 &operator*=(T s) { x *= s; y *= s; z *= s; return *this; }
  Vec3 &operator/=(T s) { x /= s; y /= s; z /= s; return *this; }

  T getX() const { return x; }
  T getY() const { return y; }
  T getZ() const { return z; }

  // serialize vector to stream
  friend std::ostream& operator<< (std::ostream& out, const Vec3& v) {
      out << "[ " << v.x << " " << v.y << " " << v.z << " ]";
      return out;
  }

  union {
      struct { T x, y, z; };
      T m[3];
  };
};

// --------------------------------------------------------------------------------

template<typename T>
class Mat2x2
{

public:
  Mat2x2() {}
  Mat2x2(T xx_, T xy_, T yx_, T yy_) : xx(xx_), xy(xy_), yx(yx_), yy(yy_) {}
  Mat2x2(T value) : xx(value), xy(value), yx(value), yy(value) {}

  ~Mat2x2() {}

  /// returns addition with v
  Mat2x2 operator+(const Mat2x2& m) const {  return Mat2x2(xx + m.xx, xy + m.xy, yx + m.yx, yy + m.yy); }

  /// returns this minus m
  Mat2x2 operator-(const Mat2x2& m) const { return Mat2x2(xx - m.xx, xy - m.xy, yx - m.yx, yy - m.yy); }

  Vec2<T> operator*(const Vec2<T>& v) const { return Vec2<T>(xx * v.x + xy * v.y, yx * v.x + yy * v.y); }

  /// return this multiplied by m
  Mat2x2 operator*(const Mat2x2& m) const { return Mat2x2(xx * m.xx + xy * m.yx, xx * m.xy + xy * m.yy,
                                                          yx * m.xx + yy * m.yx, yx * m.xy + yy * m.yy); }

  /// multiplies vector with a scalar
  Mat2x2 operator*(T s) const { return Mat2x2(xx * s, xy * s, yx * s, yy * s); }

  /// divides matrix by scalar
  Mat2x2 operator/(T s) const { return Mat2x2(xx / s, xy / s, yx / s, yy / s); }

  /// multiplies vector with a scalar
  friend Mat2x2 operator*(T s, const Mat2x2& m) { return Mat2x2(m.xx * s, m.xy * s, m.yx * s, m.yy * s); }

  static Mat2x2 identity() { return Mat2x2(1, 0, 0, 1); }

  // serialize matrix to stream
  friend std::ostream& operator<< (std::ostream& out, const Mat2x2& m) {
      out << "[ " << m.m[0] << " " << m.m[1] << std::endl
          << "  " << m.m[2] << " " << m.m[3] << " " << " ]";
      return out;
  }

  union {
      struct { T xx, xy, yx, yy; };
      T m[4];
  };

};

// --------------------------------------------------------------------------------

template<typename T>
class Mat3x3
{

public:
  Mat3x3() {}
  Mat3x3(T xx_, T xy_, T xz_, T yx_, T yy_, T yz_, T zx_, T zy_, T zz_)
    : xx(xx_), xy(xy_), xz(xz_), yx(yx_), yy(yy_), yz(yz_), zx(zx_), zy(zy_), zz(zz_) {}
  Mat3x3(T value) : xx(value), xy(value), xz(value), yx(value), yy(value), yz(value), zx(value), zy(value), zz(value) {}

  ~Mat3x3() {}

  /// returns addition with v
  Mat3x3 operator+(const Mat3x3& m) const { return Mat3x3(xx + m.xx, xy + m.xy, yx + m.yx,
                                                          yx + m.yx, yy + m.yy, yz + m.yz,
                                                          zx + m.zx, zy + m.zy, zz + m.zz); }

  /// returns this minus m
  Mat3x3 operator-(const Mat3x3& m) const { return Mat3x3(xx - m.xx, xy - m.xy, yx - m.yx,
                                                          yx - m.yx, yy - m.yy, yz - m.yz,
                                                          zx - m.zx, zy - m.zy, zz - m.zz); }

  Vec3<T> operator*(const Vec3<T>& v) const {
      return Vec3<T>(xx * v.x + xy * v.y + xz * v.z,
                     yx * v.x + yy * v.y + yz * v.z,
                     zx * v.x + zy * v.y + zz * v.z); }


  Mat3x3 operator*(const Mat3x3& m) const {
      return Mat3x3(xx * m.xx + xy * m.yx + xz * m.zx, xx * m.xy + xy * m.yy + xz * m.zy, xx * m.xz + xy * m.yz + xz * m.zz,
                    yx * m.xx + yy * m.yx + yz * m.zx, yx * m.xy + yy * m.yy + yz * m.zy, yx * m.xz + yy * m.yz + yz * m.zz,
                    zx * m.xx + zy * m.yx + zz * m.zx, zx * m.xy + zy * m.yy + zz * m.zy, zx * m.xz + zy * m.yz + zz * m.zz); }

  /// multiplies vector with a scalar
  Mat3x3 operator*(T s) const { return Mat3x3(xx * s, xy * s, xz * s,
                                              yx * s, yy * s, yz * s,
                                              zx * s, zy * s, zz * s); }

  /// divides matrix by scalar
  Mat3x3 operator/(T s) const { return Mat3x3(xx / s, xy / s, xz / s,
                                              yx / s, yy / s, yz / s,
                                              zx / s, zy / s, zz / s); }

  /// multiplies vector with a scalar
  friend Mat3x3 operator*(T s, const Mat3x3& m) { return Mat3x3(m.xx * s, m.xy * s, m.xz * s,
                                                                m.yx * s, m.yy * s, m.yz * s,
                                                                m.zx * s, m.zy * s, m.zz * s); }

  Mat3x3 transpose() const {
      return Mat3x3(xx, yx, zx,
                    xy, yy, zy,
                    xz, yz, zz);
  }

  Vec3<T> getRow(int i) const {
      return Vec3<T>(m[i*3], m[i*3+1], m[i*3+2]);
  }

  void setRPY(T roll, T pitch, T yaw) {
      T ci = cos(roll);
      T cj = cos(pitch);
      T ch = cos(yaw);
      T si = sin(roll);
      T sj = sin(pitch);
      T sh = sin(yaw);
      T cc = ci * ch;
      T cs = ci * sh;
      T sc = si * ch;
      T ss = si * sh;

      m[0] = cj * ch; m[1] = sj * sc - cs; m[2] = sj * cc + ss;
      m[3] = cj * sh; m[4] = sj * ss + cc, m[5] = sj * cs - sc;
      m[6] = -sj;     m[7] = cj * si;      m[8] = cj * ci ;
  }

  // Serialize matrix to stream
  friend std::ostream& operator<< (std::ostream& out, const Mat3x3& m) {
      out << "[ " << m.m[0] << " " << m.m[1] << " " << m.m[2] << std::endl
          << "  " << m.m[3] << " " << m.m[4] << " " << m.m[5] << std::endl
          << "  " << m.m[6] << " " << m.m[7] << " " << m.m[8] << " ]";
      return out;
  }

  union {
      struct { T xx, xy, xz, yx, yy, yz, zx, zy, zz; };
      T m[9];
  };
};

// --------------------------------------------------------------------------------

template<typename T>
class Transform3 {

public:

    Transform3() {}

    Transform3(T x, T y, T z, T roll = 0, T pitch = 0, T yaw = 0) : v_(x, y, z) {
        setRPY(roll, pitch, yaw);
    }

    Transform3(const Mat3x3<T>& r, const Vec3<T>& v) : R_(r), v_(v) {
    }

    inline Vec3<T> operator*(const Vec3<T>& v) const {
        return R_ * v + v_;
    }

    inline Transform3 operator*(const Transform3& t) const {
        return Transform3(R_ * t.R_, R_ * t.v_ + v_);
    }

    inline const Vec3<T>& getOrigin() const {
        return v_;
    }

    inline const Mat3x3<T>& getBasis() const {
        return R_;
    }

    inline Transform3 inverse() const {
        return Transform3(R_.transpose(), -v_);
    }

    void setRPY(T roll, T pitch, T yaw)  {
        R_.setRPY(roll, pitch, yaw);
    }

    friend std::ostream& operator<< (std::ostream& out, const Transform3& t) {
        out << t.R_ << std::endl << t.v_;
        return out;
    }

protected:

    Mat3x3<T> R_;
    Vec3<T> v_;

};

// --------------------------------------------------------------------------------

typedef Vec2<float> Vec2f;
typedef Vec2<double> Vec2d;
typedef Vec2<int> Vec2i;
typedef Vec2<unsigned int> Vec2u;

typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;
typedef Vec3<int> Vec3i;
typedef Vec3<unsigned int> Vec3u;

typedef Mat2x2<float> Mat2x2f;
typedef Mat2x2<double> Mat2x2d;
typedef Mat2x2<int> Mat2x2i;
typedef Mat2x2<unsigned int> Mat2x2u;

typedef Mat3x3<float> Mat3x3f;
typedef Mat3x3<double> Mat3x3d;
typedef Mat3x3<int> Mat3x3i;
typedef Mat3x3<unsigned int> Mat3x3u;

typedef Transform3<float> Transform3f;
typedef Transform3<double> Transform3d;
typedef Transform3<int> Transform3i;
typedef Transform3<unsigned int> Transform3u;

}

#endif
