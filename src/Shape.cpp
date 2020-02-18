#include "geolib/Shape.h"
#include "geolib/Box.h"

#include <geolib/serialization.h>
#include <cmath>

#include <ros/console.h>

namespace geo {

const std::string Shape::TYPE = "mesh";

Shape::Shape() : mesh_(), bounding_box_cache_valid_(false) {
}

Shape::~Shape() {

}

Shape* Shape::clone() const {
    return new Shape(*this);
}

bool Shape::intersect(const Ray &, float t0, float t1, double& distance) const {
    return false;
}

/** Check whether a point p is within distance radius of the line segment whose first vertex is described by v and second vertex by v-e
 *
 **/
bool check_linesegment(const Vector3& p, const double radius, const Vector3& v, const Vector3& e){
    double d1 = (v-p).length2();  // distance between v and p, squared
    double d2 = e.dot(v-p);  // dot product between e and v-p
    if (d2>0) {
        d2 = d2*d2 / e.length2(); // distance between v and the projection of p on e
        return d1-d2 < radius && d2 < e.length2();
    }
    return false;
}

/**
 *  @math http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.49.9172&rep=rep1&type=pdf
 **/
bool Shape::intersect(const Vector3& p, const double radius) const {
    if (p.length()-radius > mesh_.getMaxRadius()){
        return false;
    }

    if (radius > 0.0) {
        const double radius2 = radius*radius;
        // load triangles
        const std::vector<geo::Vector3>& t_points = mesh_.getPoints();
        const std::vector<TriangleI>& triangles_i = mesh_.getTriangleIs();
        for(auto it = triangles_i.begin(); it != triangles_i.end(); ++it) {
            const Vector3 &v1 = t_points[it->i1_];
            const Vector3 &v2 = t_points[it->i2_];
            const Vector3 &v3 = t_points[it->i3_];

            Vector3 e1 = v1 - v2;
            Vector3 e2 = v2 - v3;
            Vector3 e3 = v3 - v1;

            // check endpoints
            if ((v1-p).length2() < radius2) return true;
            if ((v2-p).length2() < radius2) return true;
            if ((v3-p).length2() < radius2) return true;

            // check line segments
            if (check_linesegment(p, radius, v1, e1)) return true;
            if (check_linesegment(p, radius, v2, e2)) return true;
            if (check_linesegment(p, radius, v3, e3)) return true;

            // check surface
            Vector3 norm = e1.cross(e2);
            double projected_distance2 = (p-v1).dot(norm); // projected_distance^2 = ((p-v1) dot norm)^2 / |norm|^2
            projected_distance2 = projected_distance2 * projected_distance2 / norm.length2();

            if (projected_distance2 < radius2) {
                // check that the projection falls within the triangle
                Vector3 r = p+sqrt(projected_distance2)*norm/norm.length();

                Vector3 cross1  = e1.cross(r-v2);
                Vector3 cross2  = e2.cross(r-v3);
                Vector3 cross3  = e3.cross(r-v1);

                double dot1 = cross1.dot(cross2);
                double dot2 = cross2.dot(cross3);
                double dot3 = cross3.dot(cross1);

                if (dot1 > 0 && dot2 > 0 && dot3 > 0) return true;
            }
        }
    }
    return contains(p);
}

double line_parameter(const Vector3 &D, const Vector3 &v1, const Vector3 &v2, const double d_v1, const double d_v2) {
    double p_v1 = D.dot(v1);
    double p_v2 = D.dot(v2);
    return p_v1 + (p_v2-p_v1) * d_v1/(d_v1-d_v2);
}

bool linelineintersect(const Vec2d& v1, const Vec2d& v2, const Vec2d& w1, const Vec2d& w2) {
    Vec2d ev = v2-v1;
    Vec2d ew = w2-w1;
    // check for parallel lines
    double ev_x_ew = ev.cross(ew);
    if (ev_x_ew == 0) { // lines are parallel
        if ((v1-w1).cross(ev) == 0) {// lines are collinear
            if (ev.dot(ew) > 0) { // lines are in the same diretion
                double t0 = ev.dot(w1 - v1) / ev.length2();
                double t1 = ev.dot(w2 - v1) / ev.length2();
                return t0 < 1 && t1 > 0;
            }
            else
                return false;
        }
        else
            return false;
    }
    else {
        double tv = (w1-v1).cross(ew) / ev_x_ew;
        double tw = (w1-v1).cross(ev) / ev_x_ew;
        return tv > 0 && tv < 1 && tw > 0 && tw < 1;
    }
}

void project3Dto2Dvector(Vec2d& v2d, const Vector3& v3d, int skip) {
    if (skip == 0) { // skip x, project on the (y,z) plane
        v2d.x = v3d.y;
        v2d.y = v3d.z;
    }
    else if (skip == 1) { // skip y, project on the (x,z) plane
        v2d.x = v3d.x;
        v2d.y = v3d.z;
    }
    else { // skip z, project on the (x,y) plane
        v2d.x = v3d.x;
        v2d.y = v3d.y;
    }

}

/**
 *  @math http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
 **/
bool Shape::intersect(const Pose3D& pose, const Shape& other) const {
    if (pose.t.length() > other.mesh_.getMaxRadius() + mesh_.getMaxRadius())
        return false;

    // properties of self or 1
    const std::vector<geo::Vector3>& points_1 = mesh_.getPoints();
    const std::vector<TriangleI>& triangles_1 = mesh_.getTriangleIs();
    // properties of the other or 2
    const std::vector<geo::Vector3>& points_2 = other.mesh_.getPoints();
    const std::vector<TriangleI>& triangles_2 = other.mesh_.getTriangleIs();

    // check triangle triangle intersections for all combinations
    for (auto it_1 = triangles_1.begin(); it_1 != triangles_1.end(); ++it_1) {
        Vector3 v1_1 = points_1[it_1->i1_];
        Vector3 v2_1 = points_1[it_1->i2_];
        Vector3 v3_1 = points_1[it_1->i3_];

        // transform the point to the frame of shape other
        Pose3D invpose = pose.inverse();
        v1_1 = invpose * v1_1;
        v2_1 = invpose * v2_1;
        v3_1 = invpose * v3_1;

        Vector3 N_1 = (v2_1 - v1_1).cross(v3_1 - v1_1);
        double d_1 = -N_1.dot(v1_1);

        for (auto it_2 = triangles_2.begin(); it_2 != triangles_2.end(); ++it_2) {
            Vector3 v1_2 = points_2[it_2->i1_];
            Vector3 v2_2 = points_2[it_2->i2_];
            Vector3 v3_2 = points_2[it_2->i3_];

            // define the plane of 2 as 'N_2 * X + d_2 = 0'
            Vector3 N_2 = (v2_2 - v1_2).cross(v3_2 - v1_2);
            double d_2 = -N_2.dot(v1_2);

            double d_v1_1 = N_2.dot(v1_1) + d_2;
            double d_v2_1 = N_2.dot(v2_1) + d_2;
            double d_v3_1 = N_2.dot(v3_1) + d_2;

            if ((d_v1_1 == 0 && d_v2_1 == 0 && d_v3_1 == 0) || (d_v1_1 == 0 && d_v2_1 == 0 && d_v3_1 == 0)) {
                if (N_1.dot(N_2) < 0) // triangles face opposite directions.
                    continue;

                // 2D triangle triangle test

                // convert vectors to 2D
                Vec2d t1_2d[3];
                Vec2d t2_2d[3];

                double maxN_1 = std::max(std::max(N_1.x, N_1.y), N_1.z);
                int skip = (maxN_1 == N_1.y) + 2*(maxN_1==N_2.z);

                project3Dto2Dvector(t1_2d[0], v1_1, skip);
                project3Dto2Dvector(t1_2d[1], v2_1, skip);
                project3Dto2Dvector(t1_2d[2], v3_1, skip);

                project3Dto2Dvector(t2_2d[0], v1_2, skip);
                project3Dto2Dvector(t2_2d[1], v2_2, skip);
                project3Dto2Dvector(t2_2d[2], v3_2, skip);

                // check all combinations of lines and check for intersections
                for (int i=0; i!=3; ++i) {
                    int i_next = i+1;
                    if (i_next==3) i_next=0;
                    for (int j=0; j!=3; ++j) {
                        int j_next = j+1;
                        if (j_next==3) j_next=0;
                        if (linelineintersect(t1_2d[i], t1_2d[i_next], t2_2d[j], t2_2d[j_next])) return true;
                    }
                }

                // no line intersections found
                // #TODO check if triangle 1 is contained by 2 or vice versa
                return false;
            }

            // check distance of the points of triangle 1 to the plane of triangle 2
            if ((d_v1_1 >= 0 && d_v2_1 >= 0 && d_v3_1 >= 0) || (d_v1_1 <= 0 && d_v2_1 <= 0 && d_v3_1 <= 0))
                continue;

            double d_v1_2 = N_1.dot(v1_2) + d_1;
            double d_v2_2 = N_1.dot(v2_2) + d_1;
            double d_v3_2 = N_1.dot(v3_2) + d_1;

            // check distance of the points of triangle 2 to the plane of triangle 1
            if ((d_v1_2 >= 0 && d_v2_2 >= 0 && d_v3_2 >= 0) || (d_v1_2 <= 0 && d_v2_2 <= 0 && d_v3_2 <= 0))
                continue;

            // define the line where the two planes cross 'L = D * t + O'
            Vector3 D = N_1.cross(N_2);

            // determine which line segments cross line L
            bool s_v1_1 = d_v1_1 > 0;
            bool s_v2_1 = d_v2_1 > 0;
            bool s_v3_1 = d_v3_1 > 0;
            bool s_v1_2 = d_v1_2 > 0;
            bool s_v2_2 = d_v2_2 > 0;
            bool s_v3_2 = d_v3_2 > 0;

            double t_11, t_12, t_21, t_22;

            // get intersection range of triangle 1 and L
            if (s_v1_1 == s_v2_1) {// lines 13 and 23 cross the line L
                t_11 = line_parameter(D, v1_1, v3_1, d_v1_1, d_v3_1);
                t_12 = line_parameter(D, v2_1, v3_1, d_v2_1, d_v3_1);
            }
            else {
                if (s_v1_1 == s_v3_1) {// lines 12 and 23 cross the line L
                    t_11 = line_parameter(D, v1_1, v2_1, d_v1_1, d_v2_1);
                    t_12 = line_parameter(D, v2_1, v3_1, d_v2_1, d_v3_1);
                }
                else {// lines 12 and 23 cross the line L
                    t_11 = line_parameter(D, v1_1, v2_1, d_v1_1, d_v2_1);
                    t_12 = line_parameter(D, v2_1, v3_1, d_v2_1, d_v3_1);
                }
            }

            if (s_v1_2 == s_v2_2) {// lines 13 and 23 cross the line L
                t_21 = line_parameter(D, v1_2, v3_2, d_v1_2, d_v3_2);
                t_22 = line_parameter(D, v2_2, v3_2, d_v2_2, d_v3_2);
            }
            else {
                if (s_v1_2 == s_v3_2) {// lines 12 and 23 cross the line L
                    t_21 = line_parameter(D, v1_2, v2_2, d_v1_2, d_v2_2);
                    t_22 = line_parameter(D, v2_2, v3_2, d_v2_2, d_v3_2);
                }
                else {// lines 12 and 23 cross the line L
                    t_21 = line_parameter(D, v1_2, v2_2, d_v1_2, d_v2_2);
                    t_22 = line_parameter(D, v2_2, v3_2, d_v2_2, d_v3_2);
                }
            }

            // get high and low values of both ranges
            double t_1h = std::max(t_11,t_12);
            double t_1l = std::min(t_11,t_12);
            double t_2h = std::max(t_21,t_22);
            double t_2l = std::min(t_21,t_22);

            if (t_1h > t_2l && t_1l < t_2h)
                return true;
        }
    }
    Vector3 p = pose.inverse() * points_2[0];
    return contains(p);
}


static double side_operator(Vector3& p_U, Vector3& p_V, Vector3& q_U, Vector3& q_V) {
    // calculate the side-operator of directed lines p and q given their plucker coordinates.
    // this indicates whether p and q pass eachother clockwise or counterclockwise
    return p_U.dot(q_V) + q_U.dot(p_V);
}

/**
 *  @math Let the line segment P connect points p and an arbitrary point p_out outside of the shape
 *  We count the number of intersections between P and the shape. A positive number means point p is inside the shape.
 *  We use plucker coordinates to determine whether or not a triangle intersects line segment P.
 *  more details https://members.loria.fr/SLazard/ARC-Visi3D/Pant-project/files/Line_Segment_Triangle.html
 **/
bool Shape::contains(const Vector3& p) const {
    if (p.length2() > mesh_.getSquaredMaxRadius()) {
        return false;
    }

    int intersect_count = 0;

    // determine plucker coordinates of line p
    Vector3 p_out = Vector3(1.1 * mesh_.getMaxRadius(), 0, 0);
    Vector3 p_U = p - p_out;
    Vector3 p_V = p.cross(p_out);

    // load triangles
    const std::vector<geo::Vector3>& t_points = mesh_.getPoints();
    const std::vector<TriangleI>& triangles_i = mesh_.getTriangleIs();
    for (std::vector<TriangleI>::const_iterator it = triangles_i.begin(); it != triangles_i.end(); ++it) {
        const Vector3 &v1 = t_points[it->i1_];
        const Vector3 &v2 = t_points[it->i2_];
        const Vector3 &v3 = t_points[it->i3_];

        Vector3 e1_U = v1 - v2;
        Vector3 e2_U = v2 - v3;
        Vector3 e3_U = v3 - v1;

        Vector3 e1_V = v1.cross(v2);
        Vector3 e2_V = v2.cross(v3);
        Vector3 e3_V = v3.cross(v1);

        double s1 = side_operator(p_U, p_V, e1_U, e1_V);
        double s2 = side_operator(p_U, p_V, e2_U, e2_V);
        double s3 = side_operator(p_U, p_V, e3_U, e3_V);

        // Determine whether v1, v2 and v3 circle line p (counter) clockwise.
        bool clockwise = s1 < 0 && s2 < 0 && s3 < 0;
        bool counterclockwise = s1 > 0 && s2 > 0 && s3 > 0;

        if (clockwise || counterclockwise) { // the line passes through the triangle. now check the line segment
            Vector3 l1_U = p_out - v1;
            Vector3 l2_U = v1 - p;

            Vector3 l1_V = p_out.cross(v1);
            Vector3 l2_V = v1.cross(p);

            double s4 = side_operator(l1_U, l1_V, e2_U, e2_V);
            double s5 = side_operator(l2_U, l2_V, e2_U, e2_V);

            if ((s4 > 0 && s5 < 0) || (s4 < 0 && s5 > 0)) {
                intersect_count+= counterclockwise-clockwise;
            }
        }
    }

   if (intersect_count < 0 || intersect_count > 1) {
        ROS_ERROR("intersect_count is %i, it should be 0 or 1! Is your shape constructed correctly?", intersect_count);
        return false;
    }

    return intersect_count > 0;
}

const Mesh& Shape::getMesh() const {
    return mesh_;
}

/**
 * @brief Shape::getBoundingBox returns the smallest box which includes all mesh points. Box is not rotated, but matches
 * the axis of the Shape
 * @return geo::Box of the bounding box.
 */
Box Shape::getBoundingBox() const {
    if (!bounding_box_cache_valid_)
    {
        const std::vector<geo::Vector3>& points = mesh_.getPoints();
        double x_min = 1e9, y_min = 1e9, z_min = 1e9;
        double x_max = -1e9, y_max = -1e9, z_max = -1e9;
        for (std::vector<geo::Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
        {
            x_min = std::min<double>(it->x, x_min);
            x_max = std::max<double>(it->x, x_max);
            y_min = std::min<double>(it->y, y_min);
            y_max = std::max<double>(it->y, y_max);
            z_min = std::min<double>(it->z, z_min);
            z_max = std::max<double>(it->z, z_max);
        }
        bounding_box_min_cache_ = geo::Vector3(x_min, y_min, z_min);
        bounding_box_max_cache_ = geo::Vector3(x_max, y_max, z_max);
        bounding_box_cache_valid_ = true;
    }
    return Box(bounding_box_min_cache_, bounding_box_max_cache_);
}

void Shape::setMesh(const Mesh& mesh) {
    bounding_box_cache_valid_ = false;
    mesh_ = mesh;
}

double Shape::getMaxRadius() const {
    return mesh_.getMaxRadius();
}

bool Shape::write(std::ostream& output) const {
    std::string type = "mesh    ";
    output.write(type.c_str(), type.size());

    const std::vector<geo::Vector3>& points = mesh_.getPoints();
    int p_size = points.size();
    output.write((char*)&p_size, sizeof(p_size));
    for(std::vector<geo::Vector3>::const_iterator it = points.begin(); it != points.end(); ++it) {
        const geo::Vector3& v = *it;
        double x = v.x;
        double y = v.y;
        double z = v.z;

        output.write((char*)&x, sizeof(x));
        output.write((char*)&y, sizeof(y));
        output.write((char*)&z, sizeof(z));
    }

    const std::vector<geo::TriangleI> triangles = mesh_.getTriangleIs();
    int t_size = triangles.size();
    output.write((char*)&t_size, sizeof(t_size));
    for(std::vector<geo::TriangleI>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
        const geo::TriangleI& t = *it;
        output.write((char*)&t.i1_, sizeof(t.i1_));
        output.write((char*)&t.i2_, sizeof(t.i2_));
        output.write((char*)&t.i3_, sizeof(t.i3_));
    }

    return true;
}

ShapePtr Shape::read(std::istream& input) {
    ShapePtr shape(new Shape());

    int p_size;
    input.read((char*)&p_size, sizeof(p_size));

    for(int i = 0; i < p_size; ++i) {
        double x, y, z;
        input.read((char*)&x, sizeof(x));
        input.read((char*)&y, sizeof(y));
        input.read((char*)&z, sizeof(z));
        shape->mesh_.addPoint(x, y, z);
    }

    int t_size;
    input.read((char*)&t_size, sizeof(t_size));

    for(int i = 0; i < t_size; ++i) {
        int i1, i2, i3;
        input.read((char*)&i1, sizeof(i1));
        input.read((char*)&i2, sizeof(i2));
        input.read((char*)&i3, sizeof(i3));
        shape->mesh_.addTriangle(i1, i2, i3);
    }

    return shape;
}

}
