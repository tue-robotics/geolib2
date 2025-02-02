#ifndef GEOLIB_IO_IMPORT_H_
#define GEOLIB_IO_IMPORT_H_

#include "geolib/datatypes.h"

namespace geo {

namespace io {

ShapePtr readMeshFile(const std::string& filename, const geo::Vec3& scale);

ShapePtr readMeshFile(const std::string& filename, double scale = 1.0);

}

}

#endif
