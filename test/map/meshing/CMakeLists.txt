# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

add_executable(meshing-unittest meshing_unittest.cpp)
target_link_libraries(meshing-unittest PRIVATE ${GTEST_MAIN_LIBRARIES})
add_test(NAME meshing-unittest COMMAND meshing-unittest)

