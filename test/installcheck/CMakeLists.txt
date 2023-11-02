# SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.10)
project(Supereight2-installcheck LANGUAGES CXX)

find_package(Supereight2 REQUIRED)

add_executable(installcheck "installcheck.cpp")
target_link_libraries(installcheck PRIVATE SRL::Supereight2)
