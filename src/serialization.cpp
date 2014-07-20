#include "geolib/serialization.h"

#include "geolib/Shape.h"

#include <fstream>

namespace geo {

serialization::deserializer_map serialization::deserializers_;

serialization::serialization() {

}

serialization::~serialization() {
}

bool serialization::serialize(ShapeConstPtr shape, std::ostream& output) {
    return shape->write(output);
}

bool serialization::serialize(const Shape& shape, std::ostream& output) {
    return shape.write(output);
}

ShapePtr serialization::deserialize(std::istream& input) {
    char shape_type[8];
    input.read(shape_type, 8);

    deserializer_map::const_iterator s = deserializers_.find( shape_type );
    if ( s == deserializers_.end() ) {
        // input error: don't know how to deserialize the class
        return ShapePtr();
    }
    return (s->second)( input ); // call the deserializer method
}

void serialization::registerDeserializer(const std::string& shape_type, deserialization_method method ) {
    std::string shape_type8 = "        ";
    for(unsigned int i = 0; i < std::min(shape_type.size(), shape_type8.size()); ++i) {
        shape_type8[i] = shape_type[i];
    }
    deserializers_[shape_type8] = method;
}

ShapePtr serialization::fromFile(const std::string& filename) {
    std::ifstream in;
    in.open(filename.c_str(), std::ifstream::binary);
    ShapePtr shape = geo::serialization::deserialize(in);
    in.close();
    return shape;
}

void serialization::toFile(ShapeConstPtr shape, const std::string& filename) {
    std::ofstream out;
    out.open(filename.c_str(), std::ifstream::binary);
    geo::serialization::serialize(shape, out);
    out.close();
}

void serialization::toFile(const Shape& shape, const std::string& filename) {
    std::ofstream out;
    out.open(filename.c_str(), std::ifstream::binary);
    geo::serialization::serialize(shape, out);
    out.close();}

}
