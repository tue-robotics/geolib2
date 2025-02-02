#ifndef GEOLIB_TRIANGLE_H_
#define GEOLIB_TRIANGLE_H_

#include "datatypes.h"
#include <array> 
namespace geo {

double triangleArea(const Vector3& p1, const Vector3& p2, const Vector3& p3);

class Triangle {

public:

    Triangle(const Vector3& p1_, const Vector3& p2_, const Vector3& p3_);

    virtual ~Triangle();

    inline geo::Vector3& p1() { return (*this)[0]; }

    inline const geo::Vector3& p1() const { return (*this)[0]; }

    inline geo::Vector3& p2() { return (*this)[1]; }

    inline const geo::Vector3& p2() const { return (*this)[1]; }

    inline geo::Vector3& p3() { return (*this)[2]; }

    inline const geo::Vector3& p3() const { return (*this)[2]; }

    inline geo::Vector3& operator[](const uint i) { return m[i]; }

    inline const geo::Vector3& operator[](const uint i) const { return m[i]; }

    double area() const;

    // serialize Triangle to stream
    friend std::ostream& operator<< (std::ostream& out, const Triangle& t);

    std::array<geo::Vector3, 3> m;

};

}

#endif
