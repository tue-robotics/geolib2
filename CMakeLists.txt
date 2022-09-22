cmake_minimum_required(VERSION 3.0.2)
project(geolib2)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  code_profiler
  cv_bridge
  image_geometry
  sensor_msgs
  shape_msgs
  tf
  tf2
)

SET(CMAKE_POLICY_DEFAULT_CMP0012 NEW)  # Fixed in Assimp 5.1.0, but is not released on Ubuntu Focal
find_package(ASSIMP REQUIRED)
find_package(console_bridge REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geolib
  CATKIN_DEPENDS cv_bridge shape_msgs tf
  DEPENDS console_bridge OpenCV ASSIMP
)

###########
## Build ##
###########

link_directories(${console_bridge_LIBRARY_DIRS})

include_directories(
  include
  ${ASSIMP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

if(EXISTS "/usr/include/assimp/Importer.hpp" OR EXISTS "/usr/local/include/assimp/Importer.hpp" )
    add_definitions(-DASSIMP_VERSION_3)
endif()

## Declare a cpp library
add_library(geolib  src/Box.cpp
                    src/CompositeShape.cpp
                    src/HeightMap.cpp
                    src/HeightMapNode.cpp
                    src/Mesh.cpp
                    src/Octree.cpp
                    src/OctreeNode.cpp
                    src/Ray.cpp
                    src/Shape.cpp

                    src/Triangle.cpp


                    src/shapes.cpp

                    src/sensors/DepthCamera.cpp
                    src/sensors/LaserRangeFinder.cpp

                    src/ros/msg_conversions.cpp

                    src/Importer.cpp
                    src/Exporter.cpp

                    src/serialization.cpp
                    src/visualization.cpp

                    ${HEADER_FILES})

target_link_libraries(geolib ${ASSIMP_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(show src/show.cpp)
target_link_libraries(show geolib)

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(height-map-to-shape tools/height_image_to_shape.cpp)
target_link_libraries(height-map-to-shape geolib)

add_executable(height-map-to-file tools/height_image_to_file.cpp)
target_link_libraries(height-map-to-file geolib)

# ------------------------------------------------------------------------------------------------
#                                                TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_geolib test/test_geolib.cpp)
target_link_libraries(test_geolib geolib)

add_executable(test_geolib_lrf test/test_geolib_lrf.cpp)
target_link_libraries(test_geolib_lrf geolib)

add_executable(test_matrix test/test_matrix.cpp)


# ------------------------------------------------------------------------------------------------
#                                               UNIT TESTS
# ------------------------------------------------------------------------------------------------

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_box test/test_box.cpp)
  target_link_libraries(test_box geolib)

  catkin_add_gtest(test_composite_shape test/test_composite_shape.cpp)
  target_link_libraries(test_composite_shape geolib)

  catkin_add_gtest(test_shape test/test_shape.cpp)
  target_link_libraries(test_shape geolib)

  catkin_add_gtest(test_lrf test/test_lrf.cpp)
  target_link_libraries(test_lrf geolib)
endif()
