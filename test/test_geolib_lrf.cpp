// GeoLib
#include "geolib/Box.h"
#include "geolib/sensors/LaserRangeFinder.h"

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

// LRF
unsigned int N_BEAMS = 7;
double ANGLE_MIN = -90.0/180.0*3.141592;
double ANGLE_MAX = 90.0/180.0*3.141592;

std::vector<cv::Point2i> getObjectCornerPoints(unsigned int x0, unsigned int y0,
                                               double x_obj, double y_obj, double L, double W, double obj_yaw,
                                               double pix_per_m)
{
    // Due to difference in convention
    obj_yaw = -obj_yaw;

    // Return variable
    std::vector<cv::Point2i> pts_bb(4);

    // For efficiency purposes
    double cos_th = std::cos(obj_yaw);
    double sin_th = std::sin(obj_yaw);

    // Initial corner points
    int x1 = -L/2*pix_per_m;
    int x2 = L/2*pix_per_m;
    int y1 = -W/2*pix_per_m;
    int y2 = W/2*pix_per_m;

    // Compensate for object pose
    int xt1 = x0 + x_obj*pix_per_m + cos_th*x1+sin_th*y1;
    int xt2 = x0 + x_obj*pix_per_m + cos_th*x2+sin_th*y1;
    int xt3 = x0 + x_obj*pix_per_m + cos_th*x2+sin_th*y2;
    int xt4 = x0 + x_obj*pix_per_m + cos_th*x1+sin_th*y2;
    int yt1 = y0 - (y_obj*pix_per_m - sin_th*x1+cos_th*y1);
    int yt2 = y0 - (y_obj*pix_per_m - sin_th*x2+cos_th*y1);
    int yt3 = y0 - (y_obj*pix_per_m - sin_th*x2+cos_th*y2);
    int yt4 = y0 - (y_obj*pix_per_m - sin_th*x1+cos_th*y2);

    // Add points to the vector
    pts_bb[0] = cv::Point2i(xt1, yt1);
    pts_bb[1] = cv::Point2i(xt2, yt2);
    pts_bb[2] = cv::Point2i(xt3, yt3);
    pts_bb[3] = cv::Point2i(xt4, yt4);

    return pts_bb;

}


int main(int argc, char** argv)
{

    // Determine bounding box size
    double W = 1.0;
    double L = 2.0;
    double H = 1.0;

    // Determine pose
    double x = 2;
    double y = 0.5;
    double yaw = 0.5*3.141592;

    // Geolib box
    geo::Box obj_bounding_box(geo::Vector3(-L/2, -W/2, -H/2), geo::Vector3(L/2, W/2, H/2));

    // Geolib pose
    geo::Pose3D obj_pose(x, y, 0, 0, 0, yaw);

    // Angle increment
//    double angle_incr = (ANGLE_MAX-ANGLE_MIN)/N_BEAMS;
    // I would expect the angle increment to be like this, however, in that case the rendered beams are shown incorrectly
    double angle_incr = (ANGLE_MAX-ANGLE_MIN)/(N_BEAMS-1);

    // Instantiate laser range finder model
    geo::LaserRangeFinder lrf_model;
    lrf_model.setNumBeams(N_BEAMS);
    lrf_model.setAngleLimits(ANGLE_MIN, ANGLE_MAX);
    lrf_model.setRangeLimits(0.01, 100);

    // Set render options
    geo::LaserRangeFinder::RenderOptions opt;
    opt.setMesh(obj_bounding_box.getMesh(), obj_pose);

    // Render and store ranges in a new vector
    std::vector<double> ranges;
    ranges.resize(N_BEAMS, 0);
    geo::LaserRangeFinder::RenderResult render_result(ranges);
    lrf_model.render(opt, render_result);

    // Image and associated settings
    unsigned int xsize = 501, ysize = 501, pix_per_m = 100;
    cv::Mat img(ysize, xsize, CV_8UC3, cv::Scalar(255,255,255));
    unsigned int x0 = xsize/10, y0 = ysize/2;
    cv::Scalar clr_render(0,0,255);
    cv::Scalar clr_origin(0,255,0);

    // Draw object
    std::vector<cv::Point2i> pts_bb = getObjectCornerPoints(x0, y0, x, y, L, W, yaw, pix_per_m);
    int n_pts = pts_bb.size();
    cv::Point rook_points[1][n_pts];
    for (int j=0; j<n_pts; ++j) rook_points[0][j] = pts_bb[j];\
    const cv::Point* ppt[1] = {rook_points[0]};
    int npt[] = { n_pts };
    cv::fillPoly(img, ppt, npt, 1, cv::Scalar(150,150,150));

    // Draw laser data
    double th = ANGLE_MIN;
    for (unsigned int i=0; i < render_result.ranges.size(); ++i, th += angle_incr)
    {
        double r_render = render_result.ranges[i]*pix_per_m;
        if (r_render<0.01) r_render = 10.0*pix_per_m; // make sure all beams are drawn
        // rendered scan point
        cv::circle(img, cv::Point(x0+r_render*std::cos(th), y0-r_render*std::sin(th)), 2, clr_render, 2);
        // rendered beam
        cv::line(img, cv::Point(x0,y0), cv::Point(x0+r_render*std::cos(th), y0-r_render*std::sin(th)), cv::Scalar(0,0,0));

        std::cout << th << std::endl;
    }
    // Draw origin
    cv::circle(img, cv::Point(x0,y0),3,clr_origin,2);

    // Show image
    cv::putText(img, "origin", cv::Point(0,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, clr_origin);
    cv::putText(img, "rendered point", cv::Point(0,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, clr_render);
    cv::imshow("Rendered data and object", img);
    cv::waitKey();


    return 0;
}

