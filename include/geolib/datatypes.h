#ifndef GEOLIB_DATATYPES_H_
#define GEOLIB_DATATYPES_H_

#include <string>
#include <map>
#include <vector>
#include <set>

#include <tf/transform_datatypes.h>

namespace geo {

typedef double Probability;

const static double PI = 3.1415;

//#define PI 3.1415

typedef tf::StampedTransform Transform;
typedef tf::Vector3 Vector3;
typedef tf::Point Point;
typedef tf::Quaternion Quaternion;
typedef double Time;

class Object;
typedef boost::shared_ptr<Object> ObjectPtr;
typedef boost::shared_ptr<const Object> ObjectConstPtr;

class Shape;
typedef boost::shared_ptr<Shape> ShapePtr;

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

std::ostream& operator<< (std::ostream& out, const Vector3& v);

std::ostream& operator<< (std::ostream& out, const Quaternion& q);

class Pose3D : public tf::Stamped<tf::Pose> {

public:

    Pose3D() {
        this->setOrigin(Vector3(0, 0, 0));
        this->setRotation(Quaternion(0, 0, 0, 1));
    }

    Pose3D(const Pose3D& orig) : tf::Stamped<tf::Pose>(orig) {}

    Pose3D(double x, double y, double z, const std::string& frame_id = "/map") {
        this->setOrigin(Vector3(x, y, z));
        this->setRotation(Quaternion(0, 0, 0, 1));
        this->frame_id_ = frame_id;
    }

    Pose3D(double x, double y, double z, double roll, double pitch, double yaw, const std::string& frame_id = "/map") {
        this->setOrigin(Vector3(x, y, z));
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        this->setRotation(q);
        this->frame_id_ = frame_id;
    }

    Pose3D(const Vector3& pos, const Quaternion& rot, const std::string& frame_id = "/map") {
        this->setOrigin(pos);
        this->setRotation(rot);
        this->frame_id_ = frame_id;
    }

    /*
    Pose3D operator*(const Pose3D& other) const {
        tf::Transform t = (*this) * other;
        return Pose3D(t.getOrigin(), t.getRotation());
    }
    */

    friend std::ostream& operator<< (std::ostream& out, const Pose3D& p) {
        out << "{ xyz: " << p.getOrigin();
        double roll, pitch, yaw;
        p.getBasis().getRPY(roll, pitch, yaw);
        out << ", rpy: (" << roll << ", " << pitch << ", " << yaw << ")";
        out << ", frame: '" << p.frame_id_ << "' }";
        return out;
    }

};

}

#endif
