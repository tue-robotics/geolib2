cmake_minimum_required(VERSION 3.5)
project(geolib2)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(assimp REQUIRED)
find_package(rcutils REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
  SYSTEM
  ${assimp_INCLUDE_DIRS}
  ${rcutils_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${shape_msgs_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS} 
  (/opt/ros/jazzy/include/image_geometry)
  
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
    # src/io/export.cpp
    # src/io/import.cpp
    src/ros/msg_conversions.cpp
    src/sensors/DepthCamera.cpp
    src/sensors/LaserRangeFinder.cpp
    src/serialization.cpp
    src/shapes.cpp
    src/visualization.cpp
)

target_include_directories(geolib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(geolib 
  ${assimp_LIBRARIES} 
  ${rcutils_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${image_geometry_LIBRARIES}  
  )
# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

# add_executable(height-map-to-file tools/height_image_to_file.cpp)
# target_link_libraries(height-map-to-file geolib)

# add_executable(height-map-to-shape tools/height_image_to_shape.cpp)
# target_link_libraries(height-map-to-shape geolib)

# add_executable(show tools/show.cpp)
# target_link_libraries(show geolib)

# ------------------------------------------------------------------------------------------------
#                                                TESTS
# ------------------------------------------------------------------------------------------------

# add_executable(test_geolib test/test_geolib.cpp)
# target_link_libraries(test_geolib geolib)

# add_executable(test_geolib_lrf test/test_geolib_lrf.cpp)
# target_link_libraries(test_geolib_lrf geolib)

# add_executable(test_matrix test/test_matrix.cpp)
# target_link_libraries(test_matrix geolib)

# ------------------------------------------------------------------------------------------------
#                                               INSTALL
# ------------------------------------------------------------------------------------------------



install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS geolib
  EXPORT export_geolib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#install(
#  TARGETS
    # height-map-to-file
    # height-map-to-shape
    # show
    # test_geolib
    # test_geolib_lrf
    # test_matrix
#  DESTINATION bin
#)

# ------------------------------------------------------------------------------------------------
#                                               UNIT TESTS
# ------------------------------------------------------------------------------------------------

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(test_box test/test_box.cpp)
#   target_link_libraries(test_box geolib)

#   ament_add_gtest(test_composite_shape test/test_composite_shape.cpp)
#   target_link_libraries(test_composite_shape geolib)

#   ament_add_gtest(test_shape test/test_shape.cpp)
#   target_link_libraries(test_shape geolib)

#   ament_add_gtest(test_lrf test/test_lrf.cpp)
#   target_link_libraries(test_lrf geolib)
# endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp cv_bridge image_geometry sensor_msgs shape_msgs tf2 assimp rcutils OpenCV geometry_msgs)
ament_export_targets(export_geolib)
ament_package()