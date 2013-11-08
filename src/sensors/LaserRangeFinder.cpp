#include "geolib/sensors/LaserRangeFinder.h"
#include "geolib/Shape.h"

namespace geo {

LaserRangeFinder::LaserRangeFinder() : a_min_(0), a_max_(0), num_beams_(0) {
}

LaserRangeFinder::~LaserRangeFinder() {
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        RAYTRACING
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

void LaserRangeFinder::render(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, std::vector<double>& ranges) const {
    tf::Transform t = obj_pose.inverse() * cam_pose;

    if (ranges.size() != ray_dirs_.size()) {
        ranges.resize(ray_dirs_.size(), 0);
    }

    for(unsigned int i = 0; i < ray_dirs_.size(); ++i) {
        geo::Vector3 dir_t = t.getBasis() * ray_dirs_[i];
        Ray r_t(t.getOrigin(), dir_t);

        double distance = 0;
        if (shape.intersect(r_t, range_min_, range_max_, distance)) {
            if (ranges[i] == 0 || distance < ranges[i]) {
                ranges[i] = distance;
            }
        }
    }
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        PARAMETERS
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

void LaserRangeFinder::setAngleLimits(double min, double max) {
    a_min_ = min;
    a_max_ = max;
    if (num_beams_ > 0 && a_max_ - a_min_ > 0) {
        calculateRays();
    }
}

void LaserRangeFinder::setRangeLimits(double min, double max) {
    range_min_ = min;
    range_max_ = max;
}

void LaserRangeFinder::setNumBeams(int num_beams) {
    num_beams_ = num_beams;
    if (num_beams > 0 && a_max_ - a_min_ > 0) {
        calculateRays();
    }
}

void LaserRangeFinder::calculateRays() {
    ray_dirs_.clear();
    angles_.clear();
    double a_incr = getAngleIncrement();
    double a = a_min_;
    for(int i = 0; i < num_beams_; ++i) {
        ray_dirs_.push_back(polarTo2D(-a, 1));
        angles_.push_back(a);
        a += a_incr;
    }
}

double LaserRangeFinder::getAngleMin() const {
    return a_min_;
}

double LaserRangeFinder::getAngleMax() const {
    return a_max_;
}

double LaserRangeFinder::getAngleIncrement() const {
    return (a_max_ - a_min_) / num_beams_;
}

const std::vector<double>& LaserRangeFinder::getAngles() const {
    return angles_;
}

double LaserRangeFinder::getRangeMin() const {
    return range_min_;
}

double LaserRangeFinder::getRangeMax() const {
    return range_max_;
}

int LaserRangeFinder::getNumBeams() const {
    return num_beams_;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        CONVERSIONS
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

geo::Vector3 LaserRangeFinder::polarTo2D(double angle, double range) {
    return geo::Vector3(cos(angle), sin(angle), 0) * range;
}

geo::Vector3 LaserRangeFinder::polarTo3D(const geo::Pose3D& laser_pose, double angle, double range) {
    return laser_pose.getBasis() * polarTo2D(angle, range);
}

}
