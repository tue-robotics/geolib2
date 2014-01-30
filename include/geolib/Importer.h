#ifndef GEOLIB_IMPORTER_H_
#define GEOLIB_IMPORTER_H_

#include "geolib/Shape.h"

namespace geo {

class Importer {

public:

    Importer();

    virtual ~Importer();

    static ShapePtr readMeshFile(const std::string& filename, double scale = 1.0);

};

}

#endif
