#ifndef GEOLIB_LASERRANGEFINDER_H_
#define GEOLIB_LASERRANGEFINDER_H_

#include "geolib/Ray.h"

#include <climits>
#include <vector>

namespace geo {

class Mesh;

class LaserRangeFinder {

public:

    class RenderOptions {

    public:

        void setMesh(const geo::Mesh& mesh, const geo::Pose3D& pose) {
            mesh_ = &mesh;
            pose_ = pose;
        }

        const geo::Pose3D& getPose() const { return pose_; }

        const geo::Mesh& getMesh() const { return *mesh_; }

    protected:
        const geo::Mesh* mesh_;
        geo::Pose3D pose_;

    };

    class RenderResult {

    public:

        RenderResult(std::vector<double>& ranges_) : min_i(ranges_.size()-1), max_i(0), ranges(ranges_) {}

        virtual void renderLine(const Vec2& p1, const Vec2& p2);

        virtual void renderPoint(uint index, float depth);

        uint min_i;
        uint max_i;

        std::vector<double>& ranges;

        const geo::LaserRangeFinder* lrf_;

    };

    LaserRangeFinder();

    virtual ~LaserRangeFinder();

    void render(const geo::LaserRangeFinder::RenderOptions& options, geo::LaserRangeFinder::RenderResult& res) const;

    RenderResult render(const geo::Shape& shape, const geo::Pose3D& cam_pose, const geo::Pose3D& obj_pose, std::vector<double>& ranges) const;

    void renderLine(const geo::Vec2& p1, const geo::Vec2& p2, std::vector<double>& ranges) const;

    void setAngleLimits(double min, double max);

    inline void setRangeLimits(double min, double max) { range_min_ = min; range_max_ = max; }

    void setNumBeams(uint n);

    inline double getAngleMin() const { return a_min_; }

    inline double getAngleMax() const { return a_max_; }

    /**
     * @brief Angle increment between two beams
     *
     * \f$ \frac{angle_{max} - angle_{min}}{N_{beams} - 1}\f$
     */
    double getAngleIncrement() const;

    inline const std::vector<double>& getAngles() const { return angles_; }

    inline double getRangeMin() const { return range_min_; }

    inline double getRangeMax() const { return range_max_; }

    inline uint getNumBeams() const { return num_beams_; }

    geo::Vector3 rangeToPoint(double range, uint i) const;

    const geo::Vector3 getRayDirection(uint i) const;

    bool rangesToPoints(const std::vector<double>& ranges, std::vector<geo::Vector3>& points) const;

    /**
     * @brief Get the index of the first beam with a higher angle than this beam
     * @param angle radial angle of the beam
     * @return Index of the beam, which bounded to be the number of beams [0, N_BEAMS]
     */
    uint getAngleUpperIndex(double angle) const;

    /**
     * @overload uint getAngleUpperIndex(double x, double y) const
     * @param x x-coordinate
     * @param y y-coordinate
     * @return Index of the beam, which bounded to be the number of beams [0, N_BEAMS]
     */
    uint getAngleUpperIndex(double x, double y) const;

    static geo::Vector3 polarTo2D(double angle, double range);

    static geo::Vector3 polarTo3D(const geo::Pose3D& laser_pose, double angle, double range);

    inline const std::vector<geo::Vector3>& rayDirections() const { return ray_dirs_; }

protected:

    double a_min_, a_max_;

    double range_min_, range_max_;

    uint num_beams_;

    std::vector<double> angles_;

    std::vector<geo::Vector3> ray_dirs_;

    double angle_incr_;

    // Number of beams in a half circle
    uint i_half_circle_;
    
    /**
     * @brief Get the index of the first beam with a higher angle than \p angle. Results may lie outside of the sensor range.
     * @param angle radial angle of the beam
     * @return Index of the beam
     */
    int getAngleUpperIndexRaw(double angle) const;

    /**
     * @overload int getAngleUpperIndexRaw(double x, double y) const
     * @param x x-coordinate
     * @param y y-coordinate
     * @return Index of the beam
     */
    int getAngleUpperIndexRaw(double x, double y) const;

    void calculateRays();

};

}

#endif
