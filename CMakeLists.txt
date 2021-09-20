cmake_minimum_required(VERSION 3.5...3.16)
project(se VERSION 0.1.0 LANGUAGES C CXX)

option(SE_OPENMP "Compile supereight with OpenMP" ON)
option(SE_TEST "Compile the supereight tests" ON)
option(SE_APP "Compile the supereight application" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgproc core)

# Find OpenMP, warn if disabled
if(SE_OPENMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        add_compile_options(${OpenMP_CXX_FLAGS})
        link_libraries(${OpenMP_CXX_LIBRARIES})
        message(STATUS "Compiling with OpenMP support")
    else()
        message(WARNING "OpenMP not found. Performance may be terrible.")
    endif()
else()
    message(WARNING "Building without OpenMP. Performance may be terrible.")
endif()

# Compiler options
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unknown-pragmas
    -Wno-unused-function
)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # Link against the GNU C++ library even when using clang
    # Clang also requires explicitly linking with the math library
    link_libraries(stdc++ m)
endif()

# Compile the dependencies
add_subdirectory(third_party)

# Compile the supereight library
set(LIB_NAME "supereight")
add_library(${LIB_NAME} STATIC
    "src/common/colour_utils.cpp"
    "src/common/image_utils.cpp"
    "src/common/perfstats.cpp"
    "src/common/str_utils.cpp"
    "src/common/yaml.cpp"
    "src/map/data.cpp"
    "src/map/map.cpp"
    "src/map/octant.cpp"
    "src/map/preprocessor.cpp"
    "src/map/raycaster.cpp"
    "src/sensor/ouster_lidar.cpp"
    "src/sensor/pinhole_camera.cpp"
    "src/sensor/sensor.cpp"
    "src/tracker/tracker.cpp"
)
target_include_directories(${LIB_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
    PUBLIC
      Eigen3::Eigen
      ${OpenCV_LIBS}
      SRL::Projection
)
# Add an alias so that the library can be used inside the build tree, e.g. when
# testing.
add_library(SE::Supereight ALIAS ${LIB_NAME})
# This is required so that the exported target has the name Supereight and not
# supereight.
set_target_properties(${LIB_NAME} PROPERTIES EXPORT_NAME Supereight)

# Compile the tests
if(SE_TEST)
    # enable_testing() must be called in the root CMakeLists.txt.
    enable_testing()
    add_subdirectory(test)
endif()

# Compile the app
if(SE_APP)
    add_subdirectory(app)
endif()

