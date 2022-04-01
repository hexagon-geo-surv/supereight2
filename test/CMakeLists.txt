# SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

# Setup googletest
# https://google.github.io/googletest/quickstart-cmake.html
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.11.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_subdirectory(common/image_utils)
add_subdirectory(common/str_utils)
add_subdirectory(map/keyops)
add_subdirectory(map/map)
add_subdirectory(map/meshing)
add_subdirectory(map/octree_allocator)
add_subdirectory(map/octree_visitor)
add_subdirectory(system/multires_ofusion)
# Don't add installcheck to the subdirectories, it should be built independently of the library.

