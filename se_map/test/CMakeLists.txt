cmake_minimum_required(VERSION 3.5...3.16)
project(se_map_unittests)

set(CMAKE_THREAD_PREFER_PTHREAD ON)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

add_subdirectory(keyops)
add_subdirectory(map)
add_subdirectory(meshing)
add_subdirectory(octree_allocator)
add_subdirectory(octree_visitor)
