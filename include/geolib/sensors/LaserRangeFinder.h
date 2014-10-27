#ifndef GEOLIB_LASERRANGEFINDER_H_
#define GEOLIB_LASERRANGEFINDER_H_

#include "geolib/Ray.h"

namespace geo {

class Mesh;

class LaserRangeFinder {    

public:

    class RenderOptions {

    public:

        void setMesh(const Mesh& mesh, const geo::Pose3D& pose) {
            mesh_ = &mesh;
            pose_ = pose;
        }

        const geo::Pose3D& getPose() const { return pose_; }

        const geo::Mesh& getMesh() const { return *mesh_; }

    protected:
        const Mesh* mesh_;
        geo::Pose3D pose_;

    };

    class RenderResult {

    public:

        RenderResult(std::vector<double>& ranges_) : ranges(ranges_) {}

        virtual void renderLine(const Vec2& p1, const Vec2& p2);

        virtual void renderPoint(int index, float depth);

        int min_i;
        int max_i;

        std::vector<double>& ranges;

        const LaserRangeFinder* lrf_;

    };

    LaserRangeFinder();

    virtual ~LaserRangeFinder();

    void render(const LaserRangeFinder::RenderOptions& options, LaserRangeFinder::RenderResult& res) const;

    RenderResult render(const Shape& shape, const Pose3D& cam_pose, const Pose3D& obj_pose, std::vector<double>& ranges) const;

    void renderLine(const geo::Vec2& p1, const geo::Vec2& p2, std::vector<double>& ranges) const;

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

    geo::Vector3 rangeToPoint(double range, int i) const;

    const geo::Vector3 getRayDirection(int i) const;

    bool rangesToPoints(const std::vector<double>& ranges, std::vector<geo::Vector3>& points) const;

    int getAngleUpperIndex(double angle) const;

    int getAngleUpperIndex(double x, double y) const;

    static geo::Vector3 polarTo2D(double angle, double range);

    static geo::Vector3 polarTo3D(const geo::Pose3D& laser_pose, double angle, double range);

    static double getAngle(double x, double y);

    inline const std::vector<Vector3>& rayDirections() const { return ray_dirs_; }

protected:

    double a_min_, a_max_;

    double range_min_, range_max_;

    int num_beams_;

    std::vector<double> angles_;

    std::vector<geo::Vector3> ray_dirs_;

    double slope_factor_;
    std::vector<int> slope_to_index_[8];
    int i_half_circle_;

    void calculateRays();

};

}

#endif
