cmake_minimum_required(VERSION 3.5...3.16)

add_executable(octree-visitor-unittest octree_visitor_unittest.cpp)
target_include_directories(octree-visitor-unittest PRIVATE ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(octree-visitor-unittest
    PRIVATE
    SE::Map
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
add_test(NAME octree-visitor-unittest COMMAND octree-visitor-unittest)
