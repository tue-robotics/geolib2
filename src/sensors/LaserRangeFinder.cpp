#include "geolib/sensors/LaserRangeFinder.h"
#include "geolib/Shape.h"

namespace geo {

LaserRangeFinder::LaserRangeFinder() : a_min_(0), a_max_(0), range_min_(0), range_max_(0), num_beams_(0) {
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
    int i_p1 = lrf_->getAngleUpperIndex(p1.x, p1.y);
    int i_p2 = lrf_->getAngleUpperIndex(p2.x, p2.y);

    // Get the minimum and maximum
    int i_min = std::min(i_p1, i_p2);
    int i_max = std::max(i_p1, i_p2);

    // We need to differentiate between two cases:
    // - from min to max is less than half a circle
    // - from min to max is more than half a circle (the line can be 'occluded' by the blind spot of the sensor)

    // In the latter case, we may need to render two parts

    int i_min1, i_max1, i_min2, i_max2;
    if (i_max - i_min < lrf_->i_half_circle_)
    {
        // Back-face culling: if the normal is pointing outwards, ommit this line
        if (i_p1 > i_p2)
            return;

        // Both points in the blind spot (i's are both larger number of beams), so don't render a line
        if (i_min >= lrf_->num_beams_)
            return;

        // The line is fully in view, so only need to render one part
        i_min1 = i_min;
        i_max1 = std::min(lrf_->num_beams_, i_max);

        // No second part
        i_min2 = 0;
        i_max2 = 0;

        min_i = std::min(min_i, (int)i_min);
        max_i = std::max(max_i, (int)i_max);
    }
    else
    {
        // Back-face culling: if the normal is pointing outwards, ommit this line
        if (i_p2 > i_p1)
            return;

        // We may need to draw two parts, because the line can be 'occluded' by the blind spot of the sensor
        i_min1 = i_max;
        i_max1 = lrf_->num_beams_;

        i_min2 = 0;
        i_max2 = std::min(lrf_->num_beams_, i_min);

        min_i = 0;
        max_i = lrf_->num_beams_;
    }

    // d = (q1 - ray_start) x s / (r x s)
    //   = (q1 x s) / (r x s)
    Vec2 s = p2 - p1;

    // For all beam regions found above (1 or 2 regions), calculate the intersection
    // of each beam with the line

    // Draw part 1
    for(int i = i_min1; i < i_max1; ++i)
    {
        const Vector3& r = lrf_->ray_dirs_[i];
        double d = (p1.x * s.y - p1.y * s.x) / (r.x * s.y - r.y * s.x);
        if (d > 0)
            renderPoint(i, d);
    }

    // Draw part 2
    for(int i = i_min2; i < i_max2; ++i)
    {
        const Vector3& r = lrf_->ray_dirs_[i];
        double d = (p1.x * s.y - p1.y * s.x) / (r.x * s.y - r.y * s.x);
        if (d > 0)
            renderPoint(i, d);
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserRangeFinder::RenderResult::renderPoint(int i, float d)
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
    for(unsigned int i = 0; i < points.size(); ++i)
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

    // Pre-calculate the unit direction vectors of all the rays
    double a_incr = getAngleIncrement();
    double a = a_min_;
    for(int i = 0; i < num_beams_; ++i) {
        Vector3 dir = polarTo2D(a, 1);
        ray_dirs_.push_back(dir);
        angles_.push_back(a);
        a += a_incr;
    }

    // Create a look-up table which translates slope to ray index. This way,
    // we can later on calculate the approximate polar coordinates a point on the
    // laser plane without using atan.

    // Determine the needed resolution of the look-up table (TODO: make less ad-hoc)
    slope_factor_ = 1 / tan(getAngleIncrement()) * 10;
    int N = (int)slope_factor_ + 1;

    for(unsigned int i = 0; i < 8; ++i)
        slope_to_index_[i].resize(N);

    // We divide the unit circle into 8 parts, split by 3 criteria:
    // abs(x) < abs(y)
    // x < 0
    // y < 0

    // This way we can always calculate a slope between 0 and 1 (by dividing the
    // smallest coordinate by the biggest).
    for(int j = 0; j < 8; ++j)
    {
        slope_to_index_[j].resize(N);
        for(int k = 0; k < N; ++k)
        {
            // Calculate the slope corresponding to this table entry
            double slope = (double)k / slope_factor_;

            // Calculate a virtual (x, y) cartesian point which corresponds to this table entry
            // based on the slope. Determine the part of the unit circle in which (x, y) lies
            // base on j
            double x, y;
            if (j & 1)
            {
                // x_abs > y_abs
                x = 1;
                y = slope + 1e-16; // Necessary because of 0-check in getAngle
            }
            else
            {
                // x_abs < y_abs
                x = slope + 1e-16; // Necessary because of 0-check in getAngle
                y = 1;
            }

            if (!(j & 2))
            {
                // x < 0
                x = -x;
            }

            if (!(j & 4))
            {
                // y < 0
                y = -y;
            }

            // Calculate the angle of the virtual point (x, y)
            double a = getAngle(x, y);
            double a_temp = a - a_min_;
            if (a_temp < 0)
                a_temp += 2 * M_PI;

            // Calculate the ray index based on the angle
            slope_to_index_[j][k] = a_temp / (a_max_ - a_min_) * num_beams_ + 1;
        }
    }

    i_half_circle_ = M_PI / getAngleIncrement();
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

int LaserRangeFinder::getAngleUpperIndex(double x, double y) const
{
    // Calculate the ray index corresponding to the cartesian point (x, y)
    // We use the look-up table to avoid calculating the atan of (x / y)

    double x_abs = std::abs(x);
    double y_abs = std::abs(y);

    // Calculcate slope
    double slope;
    if (x_abs < y_abs)
        slope = x_abs / y_abs;
    else
        slope = y_abs / x_abs;

    int k = slope_factor_ * slope;

    // Calculate region number (0 - 7)
    int j = (x_abs < y_abs ? 0 : 1) + (x < 0 ? 0 : 2) + (y < 0 ? 0 : 4);

    return slope_to_index_[j][k];
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
