#include "geolib/serialization.h"

#include "geolib/Shape.h"

//#include

namespace geo {

serialization::deserializer_map serialization::deserializers_;

serialization::serialization() {
}

serialization::~serialization() {
}

bool serialization::serialize(ShapeConstPtr shape, std::ostream& output) {
    return shape->write(output);
}

ShapePtr serialization::deserialize(std::istream& input) {
    char shape_type[8];
    input.read(shape_type, 8);

    std::cout << "Shape type = " << shape_type << std::endl;

    deserializer_map::const_iterator s = deserializers_.find( shape_type );
    if ( s == deserializers_.end() ) {
        // input error: don't know how to deserialize the class
        return ShapePtr();
    }
    return (s->second)( input ); // call the deserializer method
}

void serialization::registerDeserializer(const std::string& shape_type, deserialization_method method ) {
    deserializers_[shape_type] = method;
}

}
