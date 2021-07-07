cmake_minimum_required(VERSION 3.5...3.16)

add_executable(octree-allocator-unittest octree_allocator_unittest.cpp)
target_include_directories(octree-allocator-unittest PRIVATE ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(octree-allocator-unittest PRIVATE ${GTEST_MAIN_LIBRARIES})
add_test(NAME octree-allocator-unittest COMMAND octree-allocator-unittest)
