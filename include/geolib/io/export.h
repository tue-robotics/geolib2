#ifndef GEOLIB_IO_EXPORT_H_
#define GEOLIB_IO_EXPORT_H_

#include "geolib/datatypes.h"

#include <string>

namespace geo {

namespace io {

    bool writeMeshFile(const std::string& filename, const Shape& shape, std::string format = "");
}

}

#endif
