# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8...3.16)

add_executable(image-utils-io-unittest "image_utils_io_unittest.cpp")
target_link_libraries(image-utils-io-unittest PRIVATE ${GTEST_MAIN_LIBRARIES})
add_test(NAME image-utils-io-unittest COMMAND image-utils-io-unittest)

