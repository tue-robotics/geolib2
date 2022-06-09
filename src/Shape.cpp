#include "geolib/Shape.h"
#include "geolib/Box.h"

#include <geolib/serialization.h>

#include <console_bridge/console.h>

#include <cmath>
#include <stdexcept>

namespace geo {

/**
 * @brief Check whether a point p is within distance radius of the line segment. The starting point is described by @b a and the vector from the starting point to the end point @b ab
 * @param p Point to check
 * @param radius2 Radius squared to check with
 * @param a Starting point of linesegment
 * @param ab Linesegment from A to B
 * @return
 */
bool check_linesegment(const Vector3& p, const double radius2, const Vector3& a, const Vector3& ab) {
    geo::Vector3 ap = p-a;
    double d1_2 = ap.length2();  // Distance between a and p, squared
    double d2 = ab.dot(ap);  // Dot product between ab and ap; projection of p on linesegment ab

    if (d2 < 0)
        // If ab dot ap < 0, there can't be any intersection, because the orientated linesegment points away
        return false;

    double d2_2 = d2*d2 / ab.length2(); // Distance between a and the projection of p on linesegment ab, squared
    return d2_2 <= ab.length2() * (1 + 1e-9) && d1_2-d2_2 <= radius2 * (1 + 1e-9); // To prevent any numerical issues
}

class Line
{
public:
    Line(const geo::Vector3& a, const geo::Vector3& b) : a_(a), b_(b)
    {
        U_ = b_-a_;
        V_ = a_.cross(b_);
    }

    inline const geo::Vector3& a() const { return a_; }
    inline const geo::Vector3& b() const { return b_; }
    inline const geo::Vector3& U() const { return U_; }
    inline const geo::Vector3& V() const { return V_; }

protected:
    const geo::Vector3& a_, b_;
    geo::Vector3 U_, V_;
};

const std::string Shape::TYPE = "mesh";

Shape::Shape() : mesh_(), bounding_box_cache_valid_(false) {
}

Shape::~Shape() {
}

Shape* Shape::clone() const {
    return new Shape(*this);
}

bool Shape::intersect(const Ray& /*r*/, float /*t0*/, float /*t1*/, double& /*distance*/) const {
    throw std::logic_error("intersect(Ray, float, float, double) not implemented");
    return false;
}

/**
 *  Main logic:
 *  @math http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.49.9172&rep=rep1&type=pdf
 *  Projection in triangle logic:
 *  @math https://www.baeldung.com/cs/check-if-point-is-in-2d-triangle#1-mathematical-idea-2
 **/
bool Shape::intersect(const Vector3& p, const double radius) const {
    const Mesh& mesh = getMesh();
    if (p.length()-radius > mesh.getMaxRadius()) {
        return false;
    }

    if (radius > 0.) {
        const double radius2 = radius*radius;
        // load triangles
        const std::vector<geo::Vector3>& t_points = mesh.getPoints();
        const std::vector<TriangleI>& triangles_i = mesh.getTriangleIs();
        for (std::vector<TriangleI>::const_iterator it = triangles_i.cbegin(); it != triangles_i.cend(); ++it) {
            const Vector3 &v1 = t_points[it->i1_];
            const Vector3 &v2 = t_points[it->i2_];
            const Vector3 &v3 = t_points[it->i3_];

            // check endpoints
            if ((v1-p).length2() < radius2)
                return true;
            if ((v2-p).length2() < radius2)
                return true;
            if ((v3-p).length2() < radius2)
                return true;

            Vector3 e1 = v2 - v1;
            Vector3 e2 = v3 - v2;
            Vector3 e3 = v1 - v3;

            // check line segments
            if (check_linesegment(p, radius2, v1, e1))
                return true;
            if (check_linesegment(p, radius2, v2, e2))
                return true;
            if (check_linesegment(p, radius2, v3, e3))
                return true;

            // check surface
            Vector3 n = e1.cross(e2); // normal vector
            double projected_distance2 = (v1-p).dot(n); // projected_distance^2 = ((p-v1) dot n)^2 / |n|^2
            projected_distance2 = projected_distance2 * projected_distance2 / n.length2();

            if (projected_distance2 > radius2)
                // If projection distance is too big, no intersection for this triangle
                continue;

            // check that the projection falls within the triangle
            Vector3 q = p - sqrt(projected_distance2) * n.normalized();

            Vector3 cross1  = e1.cross(q-v1);
            Vector3 cross2  = e2.cross(q-v2);
            Vector3 cross3  = e3.cross(q-v3);

            double dot1 = cross1.dot(cross2);
            double dot2 = cross2.dot(cross3);
            double dot3 = cross3.dot(cross1);

            if (dot1 > 0 && dot2 > 0 && dot3 > 0)
                return true;
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
    const Mesh& mesh = getMesh();
    if (p.length2() > mesh.getSquaredMaxRadius()) {
        return false;
    }

    int intersect_count = 0;

    // determine plucker coordinates of line p
    Vector3 p_out = Vector3(1.1 * mesh.getMaxRadius(), 0, 0);
    Vector3 p_U = p - p_out;
    Vector3 p_V = p.cross(p_out);

    // load triangles
    const std::vector<geo::Vector3>& t_points = mesh.getPoints();
    const std::vector<TriangleI>& triangles_i = mesh.getTriangleIs();
    for (auto it = triangles_i.cbegin(); it != triangles_i.cend(); ++it) {
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
        CONSOLE_BRIDGE_logError("intersect_count is %i, it should be 0 or 1! Is your shape constructed correctly?", intersect_count);
        return false;
    }

    return intersect_count > 0;
}

Box Shape::getBoundingBox() const {
    if (!bounding_box_cache_valid_)
    {
        const Mesh& mesh = getMesh();
        const std::vector<geo::Vector3>& points = mesh.getPoints();
        double x_min = 1e9, y_min = 1e9, z_min = 1e9;
        double x_max = -1e9, y_max = -1e9, z_max = -1e9;
        for (auto it = points.cbegin(); it != points.cend(); ++it)
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

const Mesh& Shape::getMesh() const {
    return mesh_;
}

void Shape::setMesh(const Mesh& mesh) {
    bounding_box_cache_valid_ = false;
    mesh_ = mesh;
}

double Shape::getMaxRadius() const {
    const Mesh& mesh = getMesh();
    return mesh.getMaxRadius();
}

bool Shape::write(std::ostream& output) const {
    std::string type = "mesh    ";
    output.write(type.c_str(), type.size());

    const Mesh& mesh = getMesh();
    const std::vector<geo::Vector3>& points = mesh.getPoints();
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

    const std::vector<geo::TriangleI> triangles = mesh.getTriangleIs();
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
