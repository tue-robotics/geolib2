cmake_minimum_required(VERSION 2.8.3)
project(geolib2)

find_package(catkin REQUIRED COMPONENTS
  code_profiler
  cv_bridge
  roscpp
  tf
)

find_package(OpenCV 3 REQUIRED)
if (${OpenCV_VERSION} MATCHES "3.3.1")
  foreach(__cvcomponent ${OpenCV_LIB_COMPONENTS})
    set (__original_cvcomponent ${__cvcomponent})
    if(NOT __cvcomponent MATCHES "^opencv_")
      set(__cvcomponent opencv_${__cvcomponent})
    endif()
    if (TARGET ${__cvcomponent})
      set_target_properties(${__cvcomponent} PROPERTIES
          MAP_IMPORTED_CONFIG_DEBUG ""
          MAP_IMPORTED_CONFIG_RELEASE ""
          MAP_IMPORTED_CONFIG_RELWITHDEBINFO ""
          MAP_IMPORTED_CONFIG_MINSIZEREL ""
      )
    endif()
  endforeach(__cvcomponent)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geolib
  CATKIN_DEPENDS code_profiler cv_bridge roscpp tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)
include_directories(${OpenCV_INCLUDE_DIRS})

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

if(EXISTS "/usr/include/assimp/Importer.hpp")
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
                     
                     src/serialization.cpp
                     src/visualization.cpp
                     
                     ${HEADER_FILES})

target_link_libraries(geolib assimp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(show src/show.cpp)
target_link_libraries(show geolib)

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(height-map-to-shape tools/height_image_to_shape.cpp)
target_link_libraries(height-map-to-shape geolib)

# ------------------------------------------------------------------------------------------------
#                                                TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_geolib test/test_geolib.cpp)
target_link_libraries(test_geolib geolib assimp)

add_executable(test_geolib_lrf test/test_geolib_lrf.cpp)
target_link_libraries(test_geolib_lrf geolib assimp)

add_executable(test_matrix test/test_matrix.cpp)
