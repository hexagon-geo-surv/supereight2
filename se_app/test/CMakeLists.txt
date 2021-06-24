cmake_minimum_required(VERSION 3.5...3.16)
project(se_integrator_unittests)

set(CMAKE_THREAD_PREFER_PTHREAD ON)

find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

add_subdirectory(multires_ofusion)