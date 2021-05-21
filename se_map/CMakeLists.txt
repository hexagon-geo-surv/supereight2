cmake_minimum_required(VERSION 3.5...3.16)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Create library #
set(LIB_NAME "se-map")
add_library(${LIB_NAME} STATIC src/data.cpp src/octant.cpp src/preprocessor.cpp src/raycaster.cpp src/map.cpp)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_NAME}
  PUBLIC
    SE::Shared
    Eigen3::Eigen
)

# Add an alias so that the library can be used inside the build tree, e.g. when
# testing
add_library(SE::Map ALIAS ${LIB_NAME})
# This is required so that the exported target has the name Map and not
# se-map
set_target_properties(${LIB_NAME} PROPERTIES EXPORT_NAME Map)

add_subdirectory(test)
