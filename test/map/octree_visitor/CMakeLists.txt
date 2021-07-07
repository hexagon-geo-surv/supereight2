cmake_minimum_required(VERSION 3.5...3.16)

add_executable(octree-visitor-unittest octree_visitor_unittest.cpp)
target_link_libraries(octree-visitor-unittest PRIVATE ${GTEST_MAIN_LIBRARIES})
add_test(NAME octree-visitor-unittest COMMAND octree-visitor-unittest)

