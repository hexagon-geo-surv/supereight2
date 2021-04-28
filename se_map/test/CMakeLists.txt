cmake_minimum_required(VERSION 3.5...3.16)
project(se_map_unittests)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(keyops)
add_subdirectory(map)
add_subdirectory(meshing)
add_subdirectory(octree_allocator)
add_subdirectory(octree_visitor)
