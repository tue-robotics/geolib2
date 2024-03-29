#include "geolib/sensors/LaserRangeFinder.h"
#include "geolib/Shape.h"

#include <cmath>

namespace geo {

LaserRangeFinder::LaserRangeFinder() : a_min_(0), a_max_(0), range_min_(0), range_max_(0), num_beams_(0), angle_incr_(0) {
}

LaserRangeFinder::~LaserRangeFinder() {
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        RENDERING
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

// ----------------------------------------------------------------------------------------------------

void LaserRangeFinder::RenderResult::renderLine(const Vec2& p1, const Vec2& p2)
{
    Vec2 diff = p2 - p1;
    double line_length_sq = diff.length2();

    // Get rid of null cases
    if ((p1.x == 0 && p1.y == 0) || (p2.x == 0 && p2.y == 0) || line_length_sq == 0)
        return;


    if (lrf_->range_max_ > 0)
    {
        // Calculate distance to the line

        double t = p1.dot(diff) / -line_length_sq;

        double distance_sq;

        if (t < 0)
            distance_sq = p1.length2();
        else if (t > 1)
            distance_sq = p2.length2();
        else
            distance_sq = (p1 + t * diff).length2();

        // If too far, skip
        if (distance_sq > lrf_->range_max_ * lrf_->range_max_)
            return;
    }

    // Get the angle / beam indices based on the slope
    int i_p1 = lrf_->getAngleUpperIndexRaw(p1.x, p1.y);
    int i_p2 = lrf_->getAngleUpperIndexRaw(p2.x, p2.y);

    // Get the minimum and maximum
    int i_min = std::min<int>(i_p1, i_p2);
    int i_max = std::max<int>(i_p1, i_p2);

    // We need to differentiate between two cases:
    // - from min to max is less than half a circle
    // - from min to max is more than half a circle (the line can be 'occluded' by the blind spot of the sensor)

    // In the latter case, we may need to render two parts

    uint i_min1, i_max1, i_min2, i_max2;
    if (i_max - i_min < static_cast<int>(lrf_->i_half_circle_))
    {
        // Back-face culling: if the normal is pointing outwards, ommit this line
        if (i_p1 > i_p2)
            return;

        // Both points in the blind spot (i's are both larger number of beams), so don't render a line
        if (i_min >= static_cast<int>(lrf_->num_beams_))
            return;

        // The line is fully in view, so only need to render one part
        i_min1 = static_cast<uint>(std::max<int>(0, i_min));
        i_max1 = std::min<uint>(lrf_->num_beams_, static_cast<uint>(std::max<int>(0, i_max)));

        // No second part
        i_min2 = 0;
        i_max2 = 0;

        min_i = std::min<uint>(min_i, i_min1);
        max_i = std::max<uint>(max_i, i_max1);
    }
    else
    {
        // Back-face culling: if the normal is pointing outwards, ommit this line
        if (i_p2 > i_p1)
            return;

        // We may need to draw two parts, because the line can be 'occluded' by the blind spot of the sensor
        i_min1 = static_cast<uint>(std::max<int>(0, i_max));
        i_max1 = lrf_->num_beams_;

        i_min2 = 0;
        i_max2 = std::min<uint>(lrf_->num_beams_, static_cast<uint>(std::max<int>(0, i_min)));

        min_i = 0;
        max_i = lrf_->num_beams_;
    }

    // d = (q1 - ray_start) x s / (r x s)
    //   = (q1 x s) / (r x s)
    Vec2& s = diff;

    // For all beam regions found above (1 or 2 regions), calculate the intersection
    // of each beam with the line

    // Draw part 1
    for(uint i = i_min1; i < i_max1; ++i)
    {
        const geo::Vec2& r = lrf_->ray_dirs_[i].projectTo2d();
        double d = p1.cross(s) / r.cross(s);
        if (d > 0)
            renderPoint(i, d);
    }

    // Draw part 2
    for(uint i = i_min2; i < i_max2; ++i)
    {
        const geo::Vec2& r = lrf_->ray_dirs_[i].projectTo2d();
        double d = p1.cross(s) / r.cross(s);
        if (d > 0)
            renderPoint(i, d);
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserRangeFinder::RenderResult::renderPoint(uint i, float d)
{
    if (ranges[i] == 0 || d < ranges[i]) {
        ranges[i] = d;
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserRangeFinder::render(const LaserRangeFinder::RenderOptions& opt, LaserRangeFinder::RenderResult& res) const {
    res.min_i = 0;
    res.max_i = ray_dirs_.size();

    if (res.ranges.size() != ray_dirs_.size()) {
        res.ranges.resize(ray_dirs_.size(), 0);
    }

    res.lrf_ = this;

    const geo::Pose3D& pose = opt.getPose();

    double max_radius = opt.getMesh().getMaxRadius();
    if (max_radius > 0)
    {
        // If object is too far above or below the laser plane, do not render
        if (std::abs(pose.getOrigin().getZ()) > max_radius)
            return;

        double dist_sq = pose.getOrigin().length2();

        if (dist_sq > max_radius * max_radius) {
            // If nearest object point is certainly further away than max_range, do not render
            if (sqrt(dist_sq) - max_radius > range_max_)
                return;
        }
    }

    const std::vector<TriangleI>& triangles = opt.getMesh().getTriangleIs();
    const std::vector<Vector3>& points = opt.getMesh().getPoints();

    // transform Z-coordinates of all vertices
    std::vector<double> zs_t(points.size());
    Vector3 Rz = pose.getBasis().getRow(2);
    double z_offset = pose.getOrigin().getZ();
    for(uint i = 0; i < points.size(); ++i)
        zs_t[i] = Rz.dot(points[i]) + z_offset;

    Vector3 Rx = pose.getBasis().getRow(0);
    Vector3 Ry = pose.getBasis().getRow(1);

    // Iterate over all triangles
    for(std::vector<TriangleI>::const_iterator it_tri = triangles.begin(); it_tri != triangles.end(); ++it_tri)
    {
        double z1 = zs_t[it_tri->i1_];
        double z2 = zs_t[it_tri->i2_];
        double z3 = zs_t[it_tri->i3_];

        bool p1_under_plane = z1 < 0;
        bool p2_under_plane = z2 < 0;
        bool p3_under_plane = z3 < 0;

        // Check if not all points of the triangle are on the same side of the plane
        if (p1_under_plane != p2_under_plane || p2_under_plane != p3_under_plane)
        {
            // Transform the vertices to the sensor frame
            Vec2 p1_3d(Rx.dot(points[it_tri->i1_]) + pose.t.x, Ry.dot(points[it_tri->i1_]) + pose.t.y);
            Vec2 p2_3d(Rx.dot(points[it_tri->i2_]) + pose.t.x, Ry.dot(points[it_tri->i2_]) + pose.t.y);
            Vec2 p3_3d(Rx.dot(points[it_tri->i3_]) + pose.t.x, Ry.dot(points[it_tri->i3_]) + pose.t.y);

            // Calculate the distances of the vertices to the plane
            double z1_abs = std::abs(z1);
            double z2_abs = std::abs(z2);
            double z3_abs = std::abs(z3);

            // Calculate the intersections of the triangle edges with the plane,
            // respecting the orientation of the triangle (normal is towards or away from sensor)
            // such that later on we can do back-face culling.

            Vec2 q1, q2;
            if (p2_under_plane == p3_under_plane) {
                if (p2_under_plane)
                {
                    q2 = (p1_3d * z2_abs + p2_3d * z1_abs) / (z1_abs + z2_abs);
                    q1 = (p1_3d * z3_abs + p3_3d * z1_abs) / (z1_abs + z3_abs);
                }
                else
                {
                    q1 = (p1_3d * z2_abs + p2_3d * z1_abs) / (z1_abs + z2_abs);
                    q2 = (p1_3d * z3_abs + p3_3d * z1_abs) / (z1_abs + z3_abs);
                }
            } else if (p1_under_plane == p3_under_plane) {
                if (p1_under_plane)
                {
                    q1 = (p2_3d * z1_abs + p1_3d * z2_abs) / (z2_abs + z1_abs);
                    q2 = (p2_3d * z3_abs + p3_3d * z2_abs) / (z2_abs + z3_abs);
                }
                else
                {
                    q1 = (p2_3d * z3_abs + p3_3d * z2_abs) / (z2_abs + z3_abs);
                    q2 = (p2_3d * z1_abs + p1_3d * z2_abs) / (z2_abs + z1_abs);
                }
            }

            if (p1_under_plane == p2_under_plane) {
                if (p1_under_plane)
                {
                    q1 = (p3_3d * z2_abs + p2_3d * z3_abs) / (z3_abs + z2_abs);
                    q2 = (p3_3d * z1_abs + p1_3d * z3_abs) / (z3_abs + z1_abs);
                }
                else
                {
                    q1 = (p3_3d * z1_abs + p1_3d * z3_abs) / (z3_abs + z1_abs);
                    q2 = (p3_3d * z2_abs + p2_3d * z3_abs) / (z3_abs + z2_abs);
                }
            }

            // Render the line
            res.renderLine(geo::Vec2(q1.x, q1.y), geo::Vec2(q2.x, q2.y));
        }
    }

}

// ----------------------------------------------------------------------------------------------------

LaserRangeFinder::RenderResult LaserRangeFinder::render(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, std::vector<double>& ranges) const
{

    LaserRangeFinder::RenderOptions options;
    options.setMesh(shape.getMesh(), cam_pose.inverse() * obj_pose);

    LaserRangeFinder::RenderResult res(ranges);
    render(options, res);

    return res;
}

// ----------------------------------------------------------------------------------------------------

void LaserRangeFinder::renderLine(const geo::Vec2& p1, const geo::Vec2& p2, std::vector<double>& ranges) const
{
    LaserRangeFinder::RenderResult res(ranges);
    res.lrf_ = this;
    res.renderLine(p1, p2);
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

void LaserRangeFinder::setNumBeams(uint num_beams) {
    num_beams_ = num_beams;
    if (num_beams > 0 && a_max_ - a_min_ > 0) {
        calculateRays();
    }
}

void LaserRangeFinder::calculateRays() {
    ray_dirs_.clear();
    angles_.clear();
    angle_incr_ = (a_max_ - a_min_) / std::max<uint>(num_beams_ - 1, 1);

    ray_dirs_.resize(num_beams_);
    angles_.resize(num_beams_);

    // Pre-calculate the unit direction vectors of all the rays
    double a = a_min_;
    for(uint i = 0; i < num_beams_; ++i) {
        ray_dirs_[i] = polarTo2D(a, 1);
        angles_[i] = a;
        a += angle_incr_;
    }

    i_half_circle_ = M_PI / angle_incr_;
}

double LaserRangeFinder::getAngleIncrement() const {
    return angle_incr_;
}

uint LaserRangeFinder::getAngleUpperIndex(double angle) const {
    return std::min<uint>(num_beams_, std::max<int>(0, getAngleUpperIndexRaw(angle)));
}

uint LaserRangeFinder::getAngleUpperIndex(double x, double y) const {
    // Calculate the ray index corresponding to the cartesian point (x, y)
    return getAngleUpperIndex(atan2(y, x));
}

int LaserRangeFinder::getAngleUpperIndexRaw(double angle) const {
    return (angle - a_min_) / angle_incr_ + 1;
}

int LaserRangeFinder::getAngleUpperIndexRaw(double x, double y) const {
    // Calculate the ray index corresponding to the cartesian point (x, y)
    return getAngleUpperIndexRaw(atan2(y, x));
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                        CONVERSIONS
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

geo::Vector3 LaserRangeFinder::rangeToPoint(double range, uint i) const {
    return ray_dirs_[i] * range;
}

const geo::Vector3 LaserRangeFinder::getRayDirection(uint i) const {
    return ray_dirs_[i];
}

bool LaserRangeFinder::rangesToPoints(const std::vector<double>& ranges, std::vector<geo::Vector3>& points) const {
    if (ranges.size() != ray_dirs_.size()) {
        return false;
    }
    points.resize(ray_dirs_.size());
    for(uint i = 0; i < ray_dirs_.size(); ++i) {
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

}
