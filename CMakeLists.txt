cmake_minimum_required(VERSION 3.5...3.16)
project(se VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

option(SE_USE_OMP "Compile with OpenMP" ON)
# Find OpenMP, warn if disabled
if(SE_USE_OMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        add_compile_options(${OpenMP_CXX_FLAGS})
        link_libraries(${OpenMP_CXX_LIBRARIES})
        message(STATUS "Compiling with OpenMP support")
    else()
        message(WARNING "OpenMP not found. Performance may be terrible.")
    endif()
else(NOT SE_USE_OMP)
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

add_subdirectory(third_party)
add_subdirectory(se_map)
add_subdirectory(se_shared)
add_subdirectory(se_integrator)
add_subdirectory(se_app)