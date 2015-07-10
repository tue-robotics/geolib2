#ifndef GEOLIB_MATH_TYPES_H_
#define GEOLIB_MATH_TYPES_H_

#include <iostream>
#include <cmath>

namespace geo {

typedef double real;

// --------------------------------------------------------------------------------

template<typename T>
class Vec2T
{

public:
  Vec2T() {}
  Vec2T(T x_, T y_) : x(x_), y(y_) {}
  Vec2T(T value) : x(value), y(value) {}

  ~Vec2T() {}

  bool operator==(const Vec2T& v) const {
      return (x == v.x && y == v.y);
  }

  bool operator!=(const Vec2T& v) const {
      return !(*this == v);
  }

  /// returns dot product
  T dot(const Vec2T& v) const { return x * v.x + y * v.y; }

  /// returns addition of this and v
  Vec2T operator+(const Vec2T& v) const {  return Vec2T(x + v.x, y + v.y); }

  /// returns this minus v
  Vec2T operator-(const Vec2T& v) const { return Vec2T(x - v.x, y - v.y); }

  /// multiplies vector with a scalar
  Vec2T operator*(T s) const { return Vec2T(x * s, y * s); }

  /// divides vector by scalar
  Vec2T operator/(T s) const { return Vec2T(x / s, y / s); }

  /// multiplies vector with a scalar
  friend Vec2T operator*(T s, const Vec2T& v) { return Vec2T(v.x * s, v.y * s); }

  /// Returns the length of the vector
  T length() const { return sqrt(x * x + y * y); }

  /// Returns the squared length of the vector
  T length2() const { return x * x + y * y; }

  /// Returns the normalized version of the vector
  Vec2T normalized() const { T len = length(); return Vec2T(x / len, y / len); }

  /// Normalizes the vector
  void normalize() { T l = length(); x /= l; y /= l; }

  friend Vec2T operator-(const Vec2T& v) { return Vec2T(-v.x, -v.y); }

  Vec2T &operator+=(const Vec2T& v) { x += v.x; y += v.y; return *this; }
  Vec2T &operator-=(const Vec2T& v) { x -= v.x; y -= v.y; return *this; }
  Vec2T &operator*=(const Vec2T& v) { x *= v.x; y *= v.y; return *this; }
  Vec2T &operator/=(const Vec2T& v) { x /= v.x; y /= v.y; return *this; }

  Vec2T &operator*=(T s) { x *= s; y *= s; return *this; }
  Vec2T &operator/=(T s) { x /= s; y /= s; return *this; }

  // serialize vector to stream
  friend std::ostream& operator<< (std::ostream& out, const Vec2T& v) {
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
class Vec3T
{

public:
  Vec3T() {}
  Vec3T(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
  Vec3T(T value) : x(value), y(value), z(value) {}
  Vec3T(const T* values) { memcpy(m, values, 3 * sizeof(T)); }

  ~Vec3T() {}

  bool operator==(const Vec3T& v) const {
      return (x == v.x && y == v.y && z == v.z);
  }

  bool operator!=(const Vec3T& v) const {
      return !(*this == v);
  }

  /// returns dot product
  T dot(const Vec3T& v) const { return x * v.x + y * v.y + z * v.z; }

  /// returns cross product
  Vec3T cross(const Vec3T& v) const { return Vec3T(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x ); }

  /// returns addition with v
  Vec3T operator+(const Vec3T& v) const {  return Vec3T(x + v.x, y + v.y, z + v.z); }

  /// returns this minus v
  Vec3T operator-(const Vec3T& v) const { return Vec3T(x - v.x, y - v.y, z - v.z); }

  /// multiplies vector with a scalar
  Vec3T operator*(T s) const { return Vec3T(x * s, y * s, z * s); }

  /// divides vector by scalar
  Vec3T operator/(T s) const { return Vec3T(x / s, y / s, z / s); }

  /// multiplies vector with a scalar
  friend Vec3T operator*(T s, const Vec3T& v) { return Vec3T(v.x * s, v.y * s, v.z * s); }

  /// Returns the length of the vector
  T length() const { return sqrt(x * x + y * y + z * z); }

  /// Returns the squared length of the vector
  T length2() const { return x * x + y * y + z * z; }

  /// Returns the normalized version of the vector
  Vec3T normalized() const { T len = length(); return Vec3T(x / len, y / len, z / len); }

  /// Normalizes the vector
  void normalize() { T l = length(); x /= l; y /= l; z /= l; }

  friend Vec3T operator-(const Vec3T& v) { return Vec3T(-v.x, -v.y, -v.z); }

  Vec3T &operator+=(const Vec3T& v) { x += v.x; y += v.y; z += v.z; return *this; }
  Vec3T &operator-=(const Vec3T& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
  Vec3T &operator*=(const Vec3T& v) { x *= v.x; y *= v.y; z *= v.z; return *this; }
  Vec3T &operator/=(const Vec3T& v) { x /= v.x; y /= v.y; z /= v.z; return *this; }

  Vec3T &operator*=(T s) { x *= s; y *= s; z *= s; return *this; }
  Vec3T &operator/=(T s) { x /= s; y /= s; z /= s; return *this; }

  T getX() const { return x; }
  T getY() const { return y; }
  T getZ() const { return z; }

  // serialize vector to stream
  friend std::ostream& operator<< (std::ostream& out, const Vec3T& v) {
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
class Mat2T
{

public:
  Mat2T() {}
  Mat2T(T xx_, T xy_, T yx_, T yy_) : xx(xx_), xy(xy_), yx(yx_), yy(yy_) {}
  Mat2T(T value) : xx(value), xy(value), yx(value), yy(value) {}

  ~Mat2T() {}

  bool operator==(const Mat2T& m) const {
      return (xx == m.xx && xy == m.xy &&
              yx == m.yx && yy == m.yy );
  }

  bool operator!=(const Mat2T& m) const {
      return !(*this == m);
  }

  /// returns addition with v
  Mat2T operator+(const Mat2T& m) const {  return Mat2T(xx + m.xx, xy + m.xy, yx + m.yx, yy + m.yy); }

  /// returns this minus m
  Mat2T operator-(const Mat2T& m) const { return Mat2T(xx - m.xx, xy - m.xy, yx - m.yx, yy - m.yy); }

  Vec2T<T> operator*(const Vec2T<T>& v) const { return Vec2T<T>(xx * v.x + xy * v.y, yx * v.x + yy * v.y); }

  /// return this multiplied by m
  Mat2T operator*(const Mat2T& m) const { return Mat2T(xx * m.xx + xy * m.yx, xx * m.xy + xy * m.yy,
                                                          yx * m.xx + yy * m.yx, yx * m.xy + yy * m.yy); }

  Mat2T transpose() const {
      return Mat2T(xx, yx,
                   xy, yy);
  }

  T& operator()(int i, int j) {
      return m[i * 2  + j];
  }

  const T& operator()(int i, int j) const {
      return m[i * 2  + j];
  }

  /// multiplies vector with a scalar
  Mat2T operator*(T s) const { return Mat2T(xx * s, xy * s, yx * s, yy * s); }

  /// divides matrix by scalar
  Mat2T operator/(T s) const { return Mat2T(xx / s, xy / s, yx / s, yy / s); }

  /// multiplies vector with a scalar
  friend Mat2T operator*(T s, const Mat2T& m) { return Mat2T(m.xx * s, m.xy * s, m.yx * s, m.yy * s); }

  static Mat2T identity() { return Mat2T(1, 0, 0, 1); }

  // serialize matrix to stream
  friend std::ostream& operator<< (std::ostream& out, const Mat2T& m) {
      out << "[ " << m.m[0] << " " << m.m[1] << " ; "
                  << m.m[2] << " " << m.m[3] << " " << " ]";
      return out;
  }

  union {
      struct { T xx, xy, yx, yy; };
      T m[4];
  };

};

// --------------------------------------------------------------------------------

template<typename T>
class QuaternionT
{

public:

    QuaternionT() {}
    QuaternionT(T x_, T y_, T z_, T w_) : x(x_), y(y_), z(z_), w(w_) {}

    ~QuaternionT() {}

    bool operator==(const QuaternionT& q) const {
        return (x == q.x && y == q.y && z == q.z && w == q.w);
    }

    bool operator!=(const QuaternionT& q) const {
        return !(*this == q);
    }

    /// returns dot product
    T dot(const QuaternionT& q) const { return x * q.x + y * q.y + z * q.z + w * q.w; }

    /// returns addition with v
    QuaternionT operator+(const QuaternionT& q) const { return QuaternionT(x + q.x, y + q.y, z + q.z, w + q.w); }

    /// returns this minus v
    QuaternionT operator-(const QuaternionT& q) const { return QuaternionT(x - q.x, y - q.y, z - q.z, w - q.w); }

    T getX() const { return x; }
    T getY() const { return y; }
    T getZ() const { return z; }
    T getW() const { return w; }

    friend QuaternionT operator*(T s, const QuaternionT& q) { return QuaternionT(q.x * s, q.y * s, q.z * s, q.w * s); }

    QuaternionT operator*(T s) const { return QuaternionT(x * s, y * s, z * s, w * s); }

    /// Returns the length of the vector
    T length() const { return sqrt(x * x + y * y + z * z + w * w); }

    /// Returns the squared length of the vector
    T length2() const { return x * x + y * y + z * z + w * w; }

    /// Returns the normalized version of the vector
    QuaternionT normalized() const { T len = length(); return QuaternionT(x / len, y / len, z / len, w / len); }

    /// Normalizes the quaternion
    void normalize() { T l = length(); x /= l; y /= l; z /= l; w /= l; }

    friend std::ostream& operator<< (std::ostream& out, const QuaternionT& q) {
        out << "[ " << q.x << " " << q.y << " " << q.z << " " << q.w << " ]";
        return out;
    }

    union {
        struct { T x, y, z, w; };
        T m[4];
    };
};

// --------------------------------------------------------------------------------

template<typename T>
class Mat3T
{

public:
  Mat3T() {}

  Mat3T(T xx_, T xy_, T xz_, T yx_, T yy_, T yz_, T zx_, T zy_, T zz_)
    : xx(xx_), xy(xy_), xz(xz_), yx(yx_), yy(yy_), yz(yz_), zx(zx_), zy(zy_), zz(zz_) {}

  Mat3T(T value) : xx(value), xy(value), xz(value), yx(value), yy(value), yz(value), zx(value), zy(value), zz(value) {}

  Mat3T(const T* values) { memcpy(m, values, 9 * sizeof(T)); }

  ~Mat3T() {}

  bool operator==(const Mat3T& m) const {
      return (xx == m.xx && xy == m.xy && xz == m.xz &&
              yx == m.yx && yy == m.yy && yz == m.yz &&
              zx == m.zx && zy == m.zy && zz == m.zz);
  }

  bool operator!=(const Mat3T& m) const {
      return !(*this == m);
  }

  /// returns addition with v
  Mat3T operator+(const Mat3T& m) const { return Mat3T(xx + m.xx, xy + m.xy, xz + m.xz,
                                                          yx + m.yx, yy + m.yy, yz + m.yz,
                                                          zx + m.zx, zy + m.zy, zz + m.zz); }

  /// returns this minus m
  Mat3T operator-(const Mat3T& m) const { return Mat3T(xx - m.xx, xy - m.xy, xz - m.xz,
                                                          yx - m.yx, yy - m.yy, yz - m.yz,
                                                          zx - m.zx, zy - m.zy, zz - m.zz); }

  Vec3T<T> operator*(const Vec3T<T>& v) const {
      return Vec3T<T>(xx * v.x + xy * v.y + xz * v.z,
                     yx * v.x + yy * v.y + yz * v.z,
                     zx * v.x + zy * v.y + zz * v.z); }


  Mat3T operator*(const Mat3T& m) const {
      return Mat3T(xx * m.xx + xy * m.yx + xz * m.zx, xx * m.xy + xy * m.yy + xz * m.zy, xx * m.xz + xy * m.yz + xz * m.zz,
                    yx * m.xx + yy * m.yx + yz * m.zx, yx * m.xy + yy * m.yy + yz * m.zy, yx * m.xz + yy * m.yz + yz * m.zz,
                    zx * m.xx + zy * m.yx + zz * m.zx, zx * m.xy + zy * m.yy + zz * m.zy, zx * m.xz + zy * m.yz + zz * m.zz); }

  /// multiplies vector with a scalar
  Mat3T operator*(T s) const { return Mat3T(xx * s, xy * s, xz * s,
                                              yx * s, yy * s, yz * s,
                                              zx * s, zy * s, zz * s); }

  /// divides matrix by scalar
  Mat3T operator/(T s) const { return Mat3T(xx / s, xy / s, xz / s,
                                              yx / s, yy / s, yz / s,
                                              zx / s, zy / s, zz / s); }

  T& operator()(int i, int j) {
      return m[i * 3  + j];
  }

  const T& operator()(int i, int j) const {
      return m[i * 3  + j];
  }

  /// multiplies vector with a scalar
  friend Mat3T operator*(T s, const Mat3T& m) { return Mat3T(m.xx * s, m.xy * s, m.xz * s,
                                                                m.yx * s, m.yy * s, m.yz * s,
                                                                m.zx * s, m.zy * s, m.zz * s); }

  Mat3T transpose() const {
      return Mat3T(xx, yx, zx,
                    xy, yy, zy,
                    xz, yz, zz);
  }

  Vec3T<T> getRow(int i) const {
      return Vec3T<T>(m[i*3], m[i*3+1], m[i*3+2]);
  }

  Vec3T<T> getColumn(int i) const {
      return Vec3T<T>(m[i], m[3+i], m[6+i]);
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

  static Mat3T identity() {
      return Mat3T(1, 0, 0, 0, 1, 0, 0, 0, 1);
  }

  void getRotation(QuaternionT<T>& q) const {
      T trace = xx + yy + zz;

      if (trace > 0) {
          T s = sqrt(trace + 1);
          q.m[3] = (s / 2);
          s = 0.5 / s;

          q.m[0]=((zy - yz) * s);
          q.m[1]=((xz - zx) * s);
          q.m[2]=((yx - xy) * s);
      } else {
          int i = xx < yy ? (yy < zz ? 2 : 1) : (xx < zz ? 2 : 0);
          int j = (i + 1) % 3;
          int k = (i + 2) % 3;

          T s = sqrt(m[i*4] - m[j*4] - m[k*4] + 1);
          q.m[i] = s / 2;
          s = 0.5 / s;

          q.m[3] = (m[k*3+j] - m[j*3+k]) * s;
          q.m[j] = (m[j*3+i] + m[i*3+j]) * s;
          q.m[k] = (m[k*3+i] + m[i*3+k]) * s;
      }
  }

  void setRotation(const QuaternionT<T>& q) {
      T d = q.length2();
      T s = 2 / d;
      T xs = q.x * s,   ys = q.y * s,   zs = q.z * s;
      T wx = q.w * xs,  wy = q.w * ys,  wz = q.w * zs;
      T xx = q.x * xs,  xy = q.x * ys,  xz = q.x * zs;
      T yy = q.y * ys,  yz = q.y * zs,  zz = q.z * zs;

      m[0] = 1 - (yy + zz); m[1] = xy - wz; m[2] = xz + wy;
      m[3] = xy + wz; m[4] = 1 - (xx + zz); m[5] = yz - wx;
      m[6] = xz - wy; m[7] = yz + wx; m[8] = 1 - (xx + yy);
  }

  // Serialize matrix to stream
  friend std::ostream& operator<< (std::ostream& out, const Mat3T& m) {
      out << "[ " << m.m[0] << " " << m.m[1] << " " << m.m[2] << " ; "
                  << m.m[3] << " " << m.m[4] << " " << m.m[5] << " ; "
                  << m.m[6] << " " << m.m[7] << " " << m.m[8] << " ]";
      return out;
  }

  union {
      struct { T xx, xy, xz, yx, yy, yz, zx, zy, zz; };
      T m[9];
  };
};

// --------------------------------------------------------------------------------

template<typename T>
class Transform2T {

public:

    Transform2T() {}

    Transform2T(T x, T y, T yaw = 0) : t(x, y) {
        setRotation(yaw);
    }

    Transform2T(const Mat2T<T>& r, const Vec2T<T>& v) : R(r), t(v) {
    }

    inline Vec2T<T> operator*(const Vec2T<T>& v) const {
        return R * v + t;
    }

    inline Transform2T operator*(const Transform2T& tr) const {
        return Transform2T(R * tr.R, R * tr.t + t);
    }

    inline Transform2T inverseTimes(const Transform2T& tr) const {
        // TODO: more efficient
        return inverse() * tr;
    }

    inline const Vec2T<T>& getOrigin() const {
        return t;
    }

    inline const Mat2T<T>& getBasis() const {
        return R;
    }

    void setOrigin(const Vec2T<T>& v) { t = v; }
    void setBasis(const Mat2T<T>& r) { R = r; }

    inline Transform2T inverse() const {
        Mat2T<T> inv = R.transpose();
        return Transform2T(inv, inv * -t);
    }

    void setRotation(T yaw)  {
        T c = cos(yaw);
        T s = sin(yaw);

        R.xx = c; R.xy = -s;
        R.yx = s; R.yy = c;
    }

    T rotation() const {
        return atan2(R.yx , R.xx);
    }

    static Transform2T identity() { return Transform2T(Mat2T<T>::identity(), Vec2T<T>(0, 0));  }

    friend std::ostream& operator<< (std::ostream& out, const Transform2T& t) {
        out << "t: " << t.t << "\tR: " << t.R;
        return out;
    }

    Mat2T<T> R;
    Vec2T<T> t;

};

// --------------------------------------------------------------------------------

template<typename T>
class Transform3T {

public:

    Transform3T() {}

    Transform3T(T x, T y, T z, T roll = 0, T pitch = 0, T yaw = 0) : t(x, y, z) {
        setRPY(roll, pitch, yaw);
    }

    Transform3T(const Mat3T<T>& r, const Vec3T<T>& v) : R(r), t(v) {
    }

    inline bool operator==(const Transform3T& tr) const{
        return (R == tr.R && t == tr.t);
    }

    inline bool operator!=(const Transform3T& tr) const{
        return !(*this == tr);
    }

    inline Vec3T<T> operator*(const Vec3T<T>& v) const {
        return R * v + t;
    }

    inline Transform3T operator*(const Transform3T& tr) const {
        return Transform3T(R * tr.R, R * tr.t + t);
    }

    inline Transform3T inverseTimes(const Transform3T& tr) const {
        // TODO: more efficient
        return inverse() * tr;
    }

    inline const Vec3T<T>& getOrigin() const {
        return t;
    }

    QuaternionT<T> getQuaternion() const {
        QuaternionT<T> q;
        R.getRotation(q);
        return q;
    }

    inline const Mat3T<T>& getBasis() const {
        return R;
    }

    void setOrigin(const Vec3T<T>& v) { t = v; }
    void setBasis(const Mat3T<T>& r) { R = r; }

    inline Transform3T inverse() const {
        Mat3T<T> inv = R.transpose();
        return Transform3T(inv, inv * -t);
    }

    void setRPY(T roll, T pitch, T yaw)  {
        R.setRPY(roll, pitch, yaw);
    }

    double getYaw()  {
        QuaternionT<T> q;
        R.getRotation(q);
        return atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    }

    static Transform3T identity() { return Transform3T(Mat3T<T>::identity(), Vec3T<T>(0, 0, 0));  }

    friend std::ostream& operator<< (std::ostream& out, const Transform3T& t) {
        out << "t: " << t.t << "\tR: " << t.R;
        return out;
    }

    Mat3T<T> R;
    Vec3T<T> t;

};

// --------------------------------------------------------------------------------

typedef Vec2T<real> Vec2;
typedef Vec2T<float> Vec2f;
typedef Vec2T<double> Vec2d;
typedef Vec2T<int> Vec2i;
typedef Vec2T<unsigned int> Vec2u;

typedef Vec3T<real> Vec3;
typedef Vec3T<float> Vec3f;
typedef Vec3T<double> Vec3d;
typedef Vec3T<int> Vec3i;
typedef Vec3T<unsigned int> Vec3u;

typedef Mat2T<real> Mat2;
typedef Mat2T<float> Mat2f;
typedef Mat2T<double> Mat2d;
typedef Mat2T<int> Mat2i;
typedef Mat2T<unsigned int> Mat2u;

typedef Mat3T<real> Mat3;
typedef Mat3T<float> Mat3f;
typedef Mat3T<double> Mat3d;
typedef Mat3T<int> Mat3i;
typedef Mat3T<unsigned int> Mat3u;

typedef Transform2T<real> Transform2;
typedef Transform2T<float> Transform2f;
typedef Transform2T<double> Transform2d;
typedef Transform2T<int> Transform2i;
typedef Transform2T<unsigned int> Transform2u;

typedef Transform3T<real> Transform3;
typedef Transform3T<float> Transform3f;
typedef Transform3T<double> Transform3d;
typedef Transform3T<int> Transform3i;
typedef Transform3T<unsigned int> Transform3u;

}

#endif
