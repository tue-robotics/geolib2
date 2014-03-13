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

LaserRangeFinder::RenderResult LaserRangeFinder::raytrace(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, std::vector<double>& ranges) const {
    LaserRangeFinder::RenderResult res;
    res.min_i = 0;
    res.max_i = 0;

#ifdef GEOLIB_USE_TF
    tf::Transform t = obj_pose.inverse() * cam_pose;
#else
    Transform t = obj_pose.inverse() * cam_pose;
#endif

    if (ranges.size() != ray_dirs_.size()) {
        ranges.resize(ray_dirs_.size(), 0);
    }

    int i_min = 0;
    int i_max = (int)ray_dirs_.size();

    double max_radius = shape.getMaxRadius();
    if (max_radius > 0) {
#ifdef GEOLIB_USE_TF
        tf::Transform t_inv = t.inverse();
#else
        Transform t_inv = t.inverse();
#endif
        // If object is to far above or below the laser plane, do not render
        if (std::abs(t_inv.getOrigin().getZ()) > max_radius) {
            return res;
        }

        double dist = t_inv.getOrigin().length();

        if (dist > max_radius) {
            // If nearest object point is certainly further away than max_range, do not render
            if (dist - max_radius > range_max_) {
                return res;
            }

            double a = getAngle(t_inv.getOrigin().x(), t_inv.getOrigin().y());
            double a_limit = asin(max_radius / dist);
            double a_min = a - a_limit;
            double a_max = a + a_limit;

            i_min = std::max(0, (int)((a_min - a_min_) / getAngleIncrement()));
            i_max = std::min((int)ray_dirs_.size(), (int)((a_max - a_min_) / getAngleIncrement()));
        }
    }

    res.min_i = -1;

    for(int i = i_min; i < i_max; ++i) {
        geo::Vector3 dir_t = t.getBasis() * ray_dirs_[i];
        Ray r_t(t.getOrigin(), dir_t);

        double t1 = range_max_;
        if (ranges[i] > 0 && ranges[i] < range_max_) {
            t1 = ranges[i];
        }

        double distance = 0;
        if (shape.intersect(r_t, range_min_, t1, distance)) {
            if (ranges[i] == 0 || distance < ranges[i]) {
                ranges[i] = distance;
                if (res.min_i < 0) {
                    res.min_i = i;
                }
                res.max_i = i;
            }
        }
    }

    if (res.min_i < 0) {
        res.min_i = 0;
    }

    return res;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        RENDERING
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

LaserRangeFinder::RenderResult LaserRangeFinder::render(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, std::vector<double>& ranges) const {
    LaserRangeFinder::RenderResult res;
    res.min_i = 0;
    res.max_i = ray_dirs_.size();

#ifdef GEOLIB_USE_TF
    tf::Transform t = obj_pose.inverse() * cam_pose;
    tf::Transform t_inv = t.inverse();
#else
    Transform t = obj_pose.inverse() * cam_pose;
    Transform t_inv = t.inverse();
#endif

    if (ranges.size() != ray_dirs_.size()) {
        ranges.resize(ray_dirs_.size(), 0);
    }

    double max_radius = shape.getMaxRadius();
    if (max_radius > 0) {

        // If object is to far above or below the laser plane, do not render
        if (std::abs(t_inv.getOrigin().getZ()) > max_radius) {
            return res;
        }

        double dist = t_inv.getOrigin().length();

        if (dist > max_radius) {
            // If nearest object point is certainly further away than max_range, do not render
            if (dist - max_radius > range_max_) {
                return res;
            }
        }
    }

    const std::vector<TriangleI>& triangles = shape.getMesh().getTriangleIs();
    const std::vector<Vector3>& points = shape.getMesh().getPoints();

    // transform Z-coordinates
    std::vector<double> zs_t(points.size());
    Vector3 Rz = t_inv.getBasis().getRow(2);
    double z_offset = t_inv.getOrigin().getZ();
    for(unsigned int i = 0; i < points.size(); ++i) {
        zs_t[i] = Rz.dot(points[i]) + z_offset;
    }

    for(std::vector<TriangleI>::const_iterator it_tri = triangles.begin(); it_tri != triangles.end(); ++it_tri) {
        bool p1_under_plane = zs_t[it_tri->i1_] < 0;
        bool p2_under_plane = zs_t[it_tri->i2_] < 0;
        bool p3_under_plane = zs_t[it_tri->i3_] < 0;

        if (p1_under_plane != p2_under_plane || p2_under_plane != p3_under_plane) {

            Vector3 p1_3d = t_inv * points[it_tri->i1_];
            Vector3 p2_3d = t_inv * points[it_tri->i2_];
            Vector3 p3_3d = t_inv * points[it_tri->i3_];

            double z1 = std::abs(p1_3d.getZ());
            double z2 = std::abs(p2_3d.getZ());
            double z3 = std::abs(p3_3d.getZ());

            Vector3 q1, q2;
            if (p2_under_plane == p3_under_plane) {
                q1 = (p1_3d * z2 + p2_3d * z1) / (z1 + z2);
                q2 = (p1_3d * z3 + p3_3d * z1) / (z1 + z3);
            } else if (p1_under_plane == p3_under_plane) {
                q1 = (p2_3d * z1 + p1_3d * z2) / (z2 + z1);
                q2 = (p2_3d * z3 + p3_3d * z2) / (z2 + z3);
            } if (p1_under_plane == p2_under_plane) {
                q1 = (p3_3d * z1 + p1_3d * z3) / (z3 + z1);
                q2 = (p3_3d * z2 + p2_3d * z3) / (z3 + z2);
            }

//            int i1 = getAngleUpperIndex(q1.getX(), q1.getY());
//            int i2 = getAngleUpperIndex(q2.getX(), q2.getY());

//            int i_min = std::min(i1, i2);
//            int i_max = std::max(i1, i2);

            double a1 = getAngle(q1.getX(), q1.getY());
            double a2 = getAngle(q2.getX(), q2.getY());

            double a_min = std::min(a1, a2);
            double a_max = std::max(a1, a2);

            int i_min = getAngleUpperIndex(a_min);
            int i_max = getAngleUpperIndex(a_max);

            res.min_i = std::min(res.min_i, (int)i_min);
            res.max_i = std::max(res.max_i, (int)i_max);

            Vector3 s = q2 - q1;

            // d = (q1 - ray_start) x s / (r x s)
            //   = (q1 x s) / (r x s)


            if (a_max - a_min < M_PI) {
                // line is in front of sensor
                for(unsigned int i = i_min; (int)i < i_max; ++i) {
                    const Vector3& r = ray_dirs_[i];
                    double d = (q1.getX() * s.getY() - q1.getY() * s.getX()) / (r.getX() * s.getY() - r.getY() * s.getX());
                    if (d > 0 && (ranges[i] == 0 || d < ranges[i])) {
                        ranges[i] = d;
                    }
                }
            } else {
                // line is behind sensor
                for(unsigned int i = 0; (int)i < i_min; ++i) {
                    const Vector3& r = ray_dirs_[i];
                    double d = (q1.getX() * s.getY() - q1.getY() * s.getX()) / (r.getX() * s.getY() - r.getY() * s.getX());
                    if (d > 0 && (ranges[i] == 0 || d < ranges[i])) {
                        ranges[i] = d;
                    }
                }

                for(unsigned int i = i_max; i < ray_dirs_.size(); ++i) {
                    const Vector3& r = ray_dirs_[i];
                    double d = (q1.getX() * s.getY() - q1.getY() * s.getX()) / (r.getX() * s.getY() - r.getY() * s.getX());
                    if (d > 0 && (ranges[i] == 0 || d < ranges[i])) {
                        ranges[i] = d;
                    }
                }
            }
        }
    }

    return res;
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
    xyratio_to_index_pos_.clear();
    xyratio_to_index_neg_.clear();

    double a_incr = getAngleIncrement();
    double a = a_min_;
    for(int i = 0; i < num_beams_; ++i) {
        Vector3 dir = polarTo2D(a, 1);
        ray_dirs_.push_back(dir);
        angles_.push_back(a);
        if (dir.y() >= 0) {
            xyratio_to_index_pos_[dir.x() / dir.y()] = i;
        } else {
            xyratio_to_index_neg_[dir.x() / dir.y()] = i;
        }
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

int LaserRangeFinder::getAngleUpperIndex(double angle) const {
    int i = (angle - a_min_) / (a_max_ - a_min_) * num_beams_ + 1;
    return std::min(num_beams_, std::max(0, i));
}

int LaserRangeFinder::getAngleUpperIndex(double x, double y) const {
    if (y >= 0) {
        std::map<double, int>::const_iterator it = xyratio_to_index_pos_.lower_bound(x / y);
        if (it == xyratio_to_index_pos_.end()) {
            return num_beams_ / 2 + 1;
        }
        return it->second + 1;
    } else {
        std::map<double, int>::const_iterator it = xyratio_to_index_neg_.lower_bound(x / y);
        if (it == xyratio_to_index_neg_.end()) {
            return num_beams_ / 2 + 1;
        }
        return it->second + 1;
    }
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

geo::Vector3 LaserRangeFinder::rangeToPoint(double range, int i) const {
    return ray_dirs_[i] * range;
}

const geo::Vector3 LaserRangeFinder::getRayDirection(int i) const {
    return ray_dirs_[i];
}

bool LaserRangeFinder::rangesToPoints(const std::vector<double>& ranges, std::vector<geo::Vector3>& points) const {
    if (ranges.size() != ray_dirs_.size()) {
        return false;
    }
    points.resize(ray_dirs_.size());
    for(unsigned int i = 0; i < ray_dirs_.size(); ++i) {
        points[i] = ray_dirs_[i] * ranges[i];
    }
    return true;
}

geo::Vector3 LaserRangeFinder::polarTo2D(double angle, double range) {
    return geo::Vector3(cos(angle), sin(angle), 0) * range;
}

geo::Vector3 LaserRangeFinder::polarTo3D(const geo::Pose3D& laser_pose, double angle, double range) {
    return laser_pose.getBasis() * polarTo2D(angle, range);
}

double LaserRangeFinder::getAngle(double x, double y) {
    double a = atan(y / x);
//    double v = y / x;
//    double a = M_PI_4*v - v*(fabs(v) - 1)*(0.2447 + 0.0663*fabs(v));

    if (x < 0) {
        if (y < 0) {
            a = -M_PI + a;
        } else {
            a = M_PI + a;
        }
    }

    if (a > M_PI) {
        a -= 2 * M_PI ;
    } else if (a < -M_PI) {
        a += 2 * M_PI;
    }

    return a;
}

}
