# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.5...3.16)

# Disable warnings for third-party code
add_compile_options(-w)

# SRL Projection
option(SRL_PROJECTION_BUILD_TESTS "Enable testing" OFF)
add_subdirectory(srl_projection)

