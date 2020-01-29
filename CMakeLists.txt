cmake_minimum_required(VERSION 2.8.3)
project(geolib2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  code_profiler
  cv_bridge
  tf
  shape_msgs
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Assimp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geolib
  CATKIN_DEPENDS cv_bridge tf shape_msgs
  DEPENDS Boost OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

if(EXISTS "/usr/include/assimp/Importer.hpp" OR EXISTS "/usr/local/include/assimp/Importer.hpp" )
    add_definitions(-DASSIMP_VERSION_3)
endif()

## Declare a cpp library
add_library(geolib   src/Box.cpp
                     src/datatypes.cpp
                     src/Octree.cpp
                     src/OctreeNode.cpp
                     src/Ray.cpp
                     src/Shape.cpp
                     src/Mesh.cpp
                     src/CompositeShape.cpp
                     src/Triangle.cpp
                     src/HeightMap.cpp
                     src/HeightMapNode.cpp

                     src/shapes.cpp

                     src/sensors/DepthCamera.cpp
                     src/sensors/LaserRangeFinder.cpp

                     src/ros/msg_conversions.cpp

                     src/Importer.cpp
                     src/Exporter.cpp
                     
                     src/serialization.cpp
                     src/visualization.cpp
                     
                     ${HEADER_FILES})

target_link_libraries(geolib assimp ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

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
