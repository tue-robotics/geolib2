#include "geolib/Shape.h"
#include "geolib/Box.h"

#include <geolib/serialization.h>

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

bool Shape::intersect(const Vector3& p) const {
    if (p.length2() > mesh_.getSquaredMaxRadius()){
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
    for(std::vector<TriangleI>::const_iterator it = triangles_i.begin(); it != triangles_i.end(); ++it) {
        Vector3 v1 = t_points[it->i1_];
        Vector3 v2 = t_points[it->i2_];
        Vector3 v3 = t_points[it->i3_];

        Vector3 e1_U = v1-v2;
        Vector3 e2_U = v2 - v3;
        Vector3 e3_U = v3-v1;

        Vector3 e1_V = v1.cross(v2);
        Vector3 e2_V = v2.cross(v3);
        Vector3 e3_V = v3.cross(v1);

        double S1 = side_operator(p_U, p_V, e1_U, e1_V);
        double S2 = side_operator(p_U, p_V, e2_U, e2_V);
        double S3 = side_operator(p_U, p_V, e3_U, e3_V);

        if ((S1 < 0 && S2 < 0 && S3 < 0) || (S1 > 0 && S2 > 0 && S3 > 0)) { // the line passes through the triangle. now check the line segment
            Vector3 l1_U = p_out-v1;
            Vector3 l2_U = v1 - p;

            Vector3 l1_V = p_out.cross(v1);
            Vector3 l2_V = v1.cross(p);

            double S4 = side_operator(l1_U, l1_V, e2_U, e2_V);
            double S5 = side_operator(l2_U, l2_V, e2_U, e2_V);

            if ((S4>0 && S5<0) || (S4<0 && S5>0)){
                if (S1<0 && S2<0 && S3<0){
                    intersect_count--;
                }
                else if(S1>0 && S2>0 && S3>0){
                    intersect_count++;
                }
            }
        }
    }

    if (intersect_count<0){
        std::cout << "intersect_count lower than 0! Something went wrong!" << std::endl;
        return false;
    }

    return intersect_count > 0;
}

double Shape::side_operator(Vector3& p_U, Vector3& p_V, Vector3& q_U, Vector3& q_V){
    return p_U.dot(q_V) + q_U.dot(p_V);
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
