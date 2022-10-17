# SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

add_executable(mesh-io-unittest "mesh_io_unittest.cpp")
target_link_libraries(mesh-io-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME mesh-io-unittest COMMAND mesh-io-unittest)
