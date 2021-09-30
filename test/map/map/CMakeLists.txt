# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.5...3.16)

add_executable(map-unittest map_unittest.cpp)
target_link_libraries(map-unittest PRIVATE ${GTEST_MAIN_LIBRARIES})
add_test(NAME map-unittest COMMAND map-unittest)

