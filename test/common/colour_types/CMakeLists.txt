# SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

add_executable(colour-types-unittest "colour_types_unittest.cpp")
target_link_libraries(colour-types-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME colour-types-unittest COMMAND colour-types-unittest)

