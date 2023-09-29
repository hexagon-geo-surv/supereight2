# SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

add_executable(occupancy-integrator-unittest occupancy_integrator_unittest.cpp)
target_link_libraries(occupancy-integrator-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME occupancy-integrator-unittest COMMAND occupancy-integrator-unittest)
