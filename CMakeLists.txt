cmake_minimum_required(VERSION 3.0.2)
project(geolib2)

find_package(catkin REQUIRED COMPONENTS
  code_profiler
  cv_bridge
  roscpp
  shape_msgs
  tf
  tf2
)

find_package(ASSIMP REQUIRED)
find_package(console_bridge REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geolib
  CATKIN_DEPENDS cv_bridge roscpp shape_msgs tf
  DEPENDS console_bridge OpenCV #ASSIMP
)

###########
## Build ##
###########

link_directories(${console_bridge_LIBRARY_DIRS})

include_directories(
  include
#  ${ASSIMP_INCLUDE_DIRS} Broxen, fixed in https://github.com/assimp/assimp/pull/2161, released from 5.0.1
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
  catkin_add_gtest(test_intersect test/test_intersect.cpp)
  target_link_libraries(test_intersect geolib)
endif()
