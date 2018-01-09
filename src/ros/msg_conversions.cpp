#include "geolib/ros/msg_conversions.h"

namespace geo {

/**
 * @brief converts a geo::Mesh to shape_msgs::Mesh
 * Both use a very similar structure based on points and triangles.
 * @param m geo::Mesh as input
 * @param msg filled shape_msgs::Mesh message as output
 */
void convert(const geo::Mesh& m, shape_msgs::Mesh& msg) {
    const std::vector<Vector3>& points = m.getPoints();
    const std::vector<TriangleI>& triangles = m.getTriangleIs();

    for (std::vector<Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        geometry_msgs::Point point;
        convert(*it, point);
        msg.vertices.push_back(point);
    }

    for (std::vector<TriangleI>::const_iterator it = triangles.begin(); it != triangles.end(); ++it)
    {
        shape_msgs::MeshTriangle meshtriangle;
        convert(*it, meshtriangle);
        msg.triangles.push_back(meshtriangle);
    }
}

} // end geo namespace
