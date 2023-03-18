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

set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)  # Fixed in Assimp 5.1.0, but is not released on Ubuntu Focal
find_package(ASSIMP REQUIRED)
find_package(console_bridge REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geolib
  CATKIN_DEPENDS cv_bridge sensor_msgs shape_msgs tf
  DEPENDS ASSIMP OpenCV console_bridge
)

###########
## Build ##
###########

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
add_library(geolib
    ${HEADER_FILES}

    src/Box.cpp
    src/CompositeShape.cpp
    src/HeightMap.cpp
    src/HeightMapNode.cpp
    src/Mesh.cpp
    src/Octree.cpp
    src/OctreeNode.cpp
    src/Ray.cpp
    src/Shape.cpp
    src/Triangle.cpp
    src/io/export.cpp
    src/io/import.cpp
    src/ros/msg_conversions.cpp
    src/sensors/DepthCamera.cpp
    src/sensors/LaserRangeFinder.cpp
    src/serialization.cpp
    src/shapes.cpp
    src/visualization.cpp
)

target_link_libraries(geolib ${ASSIMP_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${OpenCV_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(height-map-to-file tools/height_image_to_file.cpp)
target_link_libraries(height-map-to-file geolib)

add_executable(height-map-to-shape tools/height_image_to_shape.cpp)
target_link_libraries(height-map-to-shape geolib)

add_executable(show tools/show.cpp)
target_link_libraries(show geolib)

# ------------------------------------------------------------------------------------------------
#                                                TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_geolib test/test_geolib.cpp)
target_link_libraries(test_geolib geolib)

add_executable(test_geolib_lrf test/test_geolib_lrf.cpp)
target_link_libraries(test_geolib_lrf geolib)

add_executable(test_matrix test/test_matrix.cpp)
target_link_libraries(test_matrix geolib)

# ------------------------------------------------------------------------------------------------
#                                               INSTALL
# ------------------------------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
  TARGETS
    geolib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    height-map-to-file
    height-map-to-shape
    show
    test_geolib
    test_geolib_lrf
    test_matrix
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# ------------------------------------------------------------------------------------------------
#                                               UNIT TESTS
# ------------------------------------------------------------------------------------------------

if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2 --ignore HEADER_OUTSIDE_PACKAGE_INCLUDE_PATH")

  catkin_add_gtest(test_box test/test_box.cpp)
  target_link_libraries(test_box geolib)

  catkin_add_gtest(test_composite_shape test/test_composite_shape.cpp)
  target_link_libraries(test_composite_shape geolib)

  catkin_add_gtest(test_shape test/test_shape.cpp)
  target_link_libraries(test_shape geolib)

  catkin_add_gtest(test_lrf test/test_lrf.cpp)
  target_link_libraries(test_lrf geolib)
endif()
