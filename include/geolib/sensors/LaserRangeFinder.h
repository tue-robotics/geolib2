#ifndef GEOLIB_LASERRANGEFINDER_H_
#define GEOLIB_LASERRANGEFINDER_H_

#include "geolib/Ray.h"

namespace geo {

class LaserRangeFinder {

public:

    LaserRangeFinder();

    virtual ~LaserRangeFinder();

    void render(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, std::vector<double>& ranges) const;

    void setAngleLimits(double min, double max);

    void setRangeLimits(double min, double max);

    void setNumBeams(int n);

    double getAngleMin() const;

    double getAngleMax() const;

    double getAngleIncrement() const;

    const std::vector<double>& getAngles() const;

    double getRangeMin() const;

    double getRangeMax() const;

    int getNumBeams() const;

    bool rangesToPoints(const std::vector<double>& ranges, std::vector<geo::Vector3>& points) const;

    static geo::Vector3 polarTo2D(double angle, double range);

    static geo::Vector3 polarTo3D(const geo::Pose3D& laser_pose, double angle, double range);

    static double getAngle(double x, double y);

protected:

    double a_min_, a_max_;

    double range_min_, range_max_;

    int num_beams_;    

    std::vector<double> angles_;

    std::vector<geo::Vector3> ray_dirs_;

    void calculateRays();

};

}

#endif
