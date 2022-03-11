# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

set(CMAKE_THREAD_PREFER_PTHREAD ON)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

# Includes and libraries common for all tests. We don't want the GTest main for
# some tests so it is linked on a per-test basis.
include_directories(${GTEST_INCLUDE_DIRS})
link_libraries(SRL::Supereight2 ${GTEST_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_subdirectory(common/image_utils)
add_subdirectory(common/str_utils)
add_subdirectory(map/keyops)
add_subdirectory(map/map)
add_subdirectory(map/meshing)
add_subdirectory(map/octree_allocator)
add_subdirectory(map/octree_visitor)
add_subdirectory(system/multires_ofusion)

