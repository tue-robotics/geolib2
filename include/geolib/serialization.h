#ifndef GEOLIB_SERIALIZATION_H_
#define GEOLIB_SERIALIZATION_H_

#include "datatypes.h"

#include <iostream>

namespace geo {

class serialization {

    typedef ShapePtr (*deserialization_method)( std::istream& );

    typedef std::map<std::string, deserialization_method> deserializer_map;

public:

    static bool serialize(ShapeConstPtr shape, std::ostream& output);

    static ShapePtr deserialize(std::istream& input);

    static void registerDeserializer(const std::string& shape_type, deserialization_method method);

protected:

    enum ShapeType {
        MESH = 0,
        OCTOMAP = 1
    };

    serialization();

    virtual ~serialization();

    static deserializer_map deserializers_;

};

}

#endif
