#ifndef GEOLIB_IMPORTER_H_
#define GEOLIB_IMPORTER_H_

#include "geolib/Shape.h"

namespace geo {

class Importer {

public:

    Importer();

    virtual ~Importer();

    static ShapePtr readMeshFile(const std::string& filename, geo::Vec3 scale);

    static ShapePtr readMeshFile(const std::string& filename, double scale)
    {
        return readMeshFile(filename, geo::Vec3(scale));
    }

    static ShapePtr readMeshFile(const std::string& filename)
    {
        return readMeshFile(filename, geo::Vec3(1.0));
    }

};

}

#endif
