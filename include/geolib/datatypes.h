#ifndef GEOLIB_DATATYPES_H_
#define GEOLIB_DATATYPES_H_

#include <string>
#include <map>
#include <vector>
#include <set>

#include "geolib/math_types.h"

#include <memory>

namespace geo {

typedef double Time;

class Shape;
typedef std::shared_ptr<Shape> ShapePtr;
typedef std::shared_ptr<const Shape> ShapeConstPtr;

class CompositeShape;
typedef std::shared_ptr<CompositeShape> CompositeShapePtr;
typedef std::shared_ptr<const CompositeShape> CompositeShapeConstPtr;

class Box;
typedef std::shared_ptr<Box> BoxPtr;
typedef std::shared_ptr<const Box> BoxConstPtr;

class Octree;
typedef std::shared_ptr<Octree> OctreePtr;
typedef std::shared_ptr<const Octree> OctreeConstPtr;

class Ray;

typedef Transform3 Transform;
typedef Transform3 Pose3D;
typedef Vec3 Vector3;
typedef Mat3 Matrix3;
typedef QuaternionT<real> Quaternion;

}

#endif
