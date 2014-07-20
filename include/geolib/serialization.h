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

    static bool serialize(const Shape& shape, std::ostream& output);

    static ShapePtr deserialize(std::istream& input);

    static ShapePtr fromFile(const std::string& filename);

    static void toFile(ShapeConstPtr shape, const std::string& filename);

    static void toFile(const Shape& shape, const std::string& filename);


    template<typename T>
    static void registerDeserializer() {
        registerDeserializer(T::TYPE, &T::read);
    }

protected:

    serialization();

    virtual ~serialization();

    static deserializer_map deserializers_;

    static void registerDeserializer(const std::string& shape_type, deserialization_method method);

};

}

#endif
