cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(geolib src/Box.cpp
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

                             src/sensors/DepthCamera.cpp
                             src/sensors/LaserRangeFinder.cpp

                             src/Importer.cpp
                             
                             src/serialization.cpp
                             src/visualization.cpp
                             
                              ${HEADER_FILES})
target_link_libraries(geolib assimp)

rosbuild_add_executable(show src/show.cpp)
target_link_libraries(show geolib)

# Tests
rosbuild_add_executable(test_geolib test/test_geolib.cpp)
target_link_libraries(test_geolib geolib assimp)

rosbuild_add_executable(test_matrix test/test_matrix.cpp)

