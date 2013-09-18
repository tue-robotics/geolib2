#ifndef _Triangle_H_
#define _Triangle_H_

#include "datatypes.h"

namespace vwm {

class Triangle {

public:

    Triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3);

    virtual ~Triangle();

    Vector3 p1_, p2_, p3_;

};

}

#endif
