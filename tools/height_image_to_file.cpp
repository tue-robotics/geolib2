#include <geolib/HeightMap.h>
#include <geolib/io/export.h>

#include <opencv2/highgui/highgui.hpp>

#include <vector>

int main(int argc, char **argv) {

    // Parse command-line arguments
    if (argc < 3 || argc > 7) {
        std::cerr << "Usage: height_map_to_file INPUT_IMAGE OUTPUT_FILE RESOLUTION [BLOCK_HEIGHT] [ORIGIN_X ORIGIN_Y]" << std::endl;
        return 1;
    }

    std::string filename_img = argv[1];
    std::string output_file = argv[2];

    double resolution = 0.2;
    if (argc > 3) {
        resolution = atof(argv[3]);
    }

    double block_height = 1;
    if (argc > 4) {
        block_height = atof(argv[4]);
    }

    double origin_x = 0, origin_y = 0;
    if (argc > 6) {
        origin_x = atof(argv[5]);
        origin_y = atof(argv[6]);
    }

    cv::Mat image = cv::imread(filename_img, cv::IMREAD_GRAYSCALE);   // Read the file

    std::vector<std::vector<double> > map;

    if (image.data ) {
        map.resize(image.cols);
        for(int x = 0; x < image.cols; ++x) {
            map[x].resize(image.rows);
            for(int y = 0; y < image.rows; ++y) {
                map[x][image.rows - y - 1] = block_height - (double)image.at<unsigned char>(y, x) / 255 * block_height;
            }
        }
        std::cout << "Loaded height map " << filename_img << std::endl;
    } else {
        std::cout << "Could not load height map " << filename_img << std::endl;
        return 1;
    }

    geo::HeightMap hmap = geo::HeightMap::fromGrid(map, resolution);

    // Transform according to given origin
    geo::Pose3D transform(origin_x, origin_y, 0, 0, 0, 0);
    geo::Mesh mesh_transformed = hmap.getMesh().getTransformed(transform);

    geo::Shape shape;
    shape.setMesh(mesh_transformed);

    geo::io::writeMeshFile(output_file, shape);

    std::cout << mesh_transformed.getTriangleIs().size() << " triangles and " << mesh_transformed.getPoints().size() <<
                 " points saved to '" << output_file << "'." << std::endl;
    return 0;
}
