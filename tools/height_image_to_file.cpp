#include "geolib/datatypes.h"
#include "geolib/Mesh.h"
#include "geolib/Shape.h"
#include <cstdlib>
#include <geolib/HeightMap.h>
#include <geolib/io/export.h>

#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgcodecs.hpp>
#include <ostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{

    // Parse command-line arguments
    if (argc < 3 || argc > 7)
    {
        std::cerr << "Usage: height_map_to_file INPUT_IMAGE OUTPUT_FILE "
                     "RESOLUTION [BLOCK_HEIGHT] [ORIGIN_X ORIGIN_Y]"
                  << '\n';
        return 1;
    }

    std::string const filename_img = argv[1];
    std::string const output_file = argv[2];

    double resolution = 0.2;
    if (argc > 3)
    {
        resolution = atof(argv[3]);
    }

    double block_height = 1;
    if (argc > 4)
    {
        block_height = atof(argv[4]);
    }

    double origin_x = 0;
    double origin_y = 0;
    if (argc > 6)
    {
        origin_x = atof(argv[5]);
        origin_y = atof(argv[6]);
    }

    cv::Mat image = cv::imread(filename_img, cv::IMREAD_GRAYSCALE); // Read the file

    std::vector<std::vector<double>> map;

    if (image.data)
    {
        map.resize(image.cols);
        for (int x = 0; x < image.cols; ++x)
        {
            map[x].resize(image.rows);
            for (int y = 0; y < image.rows; ++y)
            {
                map[x][image.rows - y - 1] =
                    block_height - (static_cast<double>(image.at<unsigned char>(y, x)) / 255 * block_height);
            }
        }
        std::cout << "Loaded height map " << filename_img << '\n';
    }
    else
    {
        std::cout << "Could not load height map " << filename_img << '\n';
        return 1;
    }

    geo::HeightMap const hmap = geo::HeightMap::fromGrid(map, resolution);

    // Transform according to given origin
    geo::Pose3D const transform(origin_x, origin_y, 0, 0, 0, 0);
    geo::Mesh const mesh_transformed = hmap.getMesh().getTransformed(transform);

    geo::Shape shape;
    shape.setMesh(mesh_transformed);

    geo::io::writeMeshFile(output_file, shape);

    std::cout << mesh_transformed.getTriangleIs().size() << " triangles and " << mesh_transformed.getPoints().size()
              << " points saved to '" << output_file << "'." << '\n';
    return 0;
}
