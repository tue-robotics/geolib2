cmake_minimum_required(VERSION 3.5)
project(geolib2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror=all)
  add_compile_options(-Wextra -Werror=extra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)  # Fixed in Assimp 5.1.0, but is not released on Ubuntu Focal
find_package(assimp REQUIRED)
find_package(console_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

if(EXISTS "/usr/include/assimp/Importer.hpp" OR EXISTS "/usr/local/include/assimp/Importer.hpp")
  add_definitions(-DASSIMP_VERSION_3)
endif()

## Declare a cpp library
add_library(geolib SHARED
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

target_include_directories(geolib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(geolib SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(geolib SYSTEM PRIVATE
  ${console_bridge_INCLUDE_DIRS}
)

ament_target_dependencies(geolib
  geometry_msgs
  image_geometry
  sensor_msgs
  shape_msgs
  tf2
)

target_link_libraries(geolib
  assimp::assimp
  ${console_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

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

find_package(code_profiler REQUIRED)

add_executable(test_geolib test/test_geolib.cpp)
target_link_libraries(test_geolib geolib code_profiler)

add_executable(test_geolib_lrf test/test_geolib_lrf.cpp)
target_link_libraries(test_geolib_lrf geolib)

add_executable(test_matrix test/test_matrix.cpp)
target_link_libraries(test_matrix geolib)

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
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    height-map-to-file
    height-map-to-shape
    show
    test_geolib
    test_geolib_lrf
    test_matrix
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------------------------
#                                               EXPORT
# ------------------------------------------------------------------------------------------------

ament_export_targets(export_geolib HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs image_geometry sensor_msgs shape_msgs tf2 OpenCV)

# ------------------------------------------------------------------------------------------------
#                                               UNIT TESTS
# ------------------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(tue_lint_config REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_clang_format(
    CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-format
    --clang-format-version=21
  )
  ament_clang_tidy(
    CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-tidy
    --clang-tidy-version=21
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR}/compile_commands.json)
  ament_lint_cmake(MAX_LINE_LENGTH 120 "--filter=
    +convention/filename,
    +linelength,
    +package/consistency,
    +package/stdargs,
    +readability/logic,
    +readability/mixedcase,
    +readability/wonkycase,
    +syntax,+whitespace/eol,
    +whitespace/extra,
    +whitespace/indent,
    +whitespace/mismatch,
    +whitespace/newline,
    +whitespace/tabs"
  )
  ament_xmllint(MAX_LINE_LENGTH 120)

  ament_add_gtest(test_box test/test_box.cpp)
  target_link_libraries(test_box geolib)

  ament_add_gtest(test_composite_shape test/test_composite_shape.cpp)
  target_link_libraries(test_composite_shape geolib)

  ament_add_gtest(test_shape test/test_shape.cpp)
  target_link_libraries(test_shape geolib)

  ament_add_gtest(test_lrf test/test_lrf.cpp)
  target_link_libraries(test_lrf geolib)
endif()

ament_package()
