# SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.16)

add_executable(octree-visitor-unittest octree_visitor_unittest.cpp)
target_link_libraries(octree-visitor-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME octree-visitor-unittest COMMAND octree-visitor-unittest)
