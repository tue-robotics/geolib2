#ifndef GEOLIB_EXPORTER_H_
#define GEOLIB_EXPORTER_H_

#include "geolib/Shape.h"

namespace geo {

class Exporter {

public:

    Exporter();

    virtual ~Exporter();

    bool writeMeshFile(const std::string& filename, const Shape& shape, double scale = 1.0);
};

}

#endif
