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
            Vector3 e3 = v1 - v3;

            // check endpoints
            if ((v1-p).length2() < radius2) return true;
            if ((v2-p).length2() < radius2) return true;
            if ((v3-p).length2() < radius2) return true;

            // check line segments
            double d1 = (v1-p).length2();  // distance between v1 and p squared
            double d2 = e1.dot(v1-p);  // dot product between e1 and v1-p
            if (d2>0) {
                d2 = d2*d2 / e1.length2(); // distance between v1 and the projection of p on e1
                if (d1-d2 < radius2 && d2 < e1.length2()) return true;
            }

            d2 = e3.dot(v1-p);  // dot product between e3 and v1-p
            if (d2>0) {
                d2 = d2*d2 / e3.length2(); // distance between v1 and the projection of p on e3
                if (d1-d2 < radius2 && d2 < e1.length2()) return true;
            }

            d1 = (v2-p).length2();  // distance between v2 and p squared
            d2 = e2.dot(v2-p);  // dot product between e2 and v2-p
            if (d2>0) {
                d2 = d2*d2 / e2.length2(); // distance between v2 and the projection of p on e2
                if (d1-d2 < radius2 && d2 < e2.length2()) return true;
            }

            // check surface
            Vector3 norm = e1.cross(e2);
            double projected_distance2 = (p-v1).dot(norm); // projected_distance^2 = ((p-v1) dot norm)^2 / |norm|^2
            projected_distance2 = projected_distance2 * projected_distance2 / norm.length2();

            if (projected_distance2 < radius2) {
                // check that the projection falls within the triangle
                Vector3 r = p+sqrt(projected_distance2)*norm/norm.length();

                Vector3 cross1  = e1.cross(r-v2);
                Vector3 cross2  = e2.cross(r-v3);
                Vector3 cross3  = e3.cross(r-v3);

                double dot1 = cross1.dot(cross2);
                double dot2 = cross1.dot(cross3);
                double dot3 = cross2.dot(cross3);

                if (dot1 > 0 && dot2 < 0 && dot3 < 0) return true;
            }
        }
    }
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
