#ifndef GEOLIB_DATATYPES_H_
#define GEOLIB_DATATYPES_H_

#include <string>
#include <map>
#include <vector>
#include <set>

#include "geolib/math_types.h"
#include <boost/shared_ptr.hpp>

namespace geo {

typedef double Time;

class Object;
typedef boost::shared_ptr<Object> ObjectPtr;
typedef boost::shared_ptr<const Object> ObjectConstPtr;

class Shape;
typedef boost::shared_ptr<Shape> ShapePtr;
typedef boost::shared_ptr<const Shape> ShapeConstPtr;

class Box;
typedef boost::shared_ptr<Box> BoxPtr;

class Plugin;
typedef boost::shared_ptr<Plugin> PluginPtr;

class WorldView;
typedef boost::shared_ptr<WorldView> WorldViewPtr;
typedef boost::shared_ptr<const WorldView> WorldViewConstPtr;

class Octree;
typedef boost::shared_ptr<Octree> OctreePtr;
typedef boost::shared_ptr<const Octree> OctreeConstPtr;

class Ray;

typedef Transform3 Transform;
typedef Transform3 Pose3D;
typedef Vec3 Vector3;
typedef Mat3 Matrix3;
typedef QuaternionT<real> Quaternion;

}

#endif
