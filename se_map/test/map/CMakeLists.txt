cmake_minimum_required(VERSION 3.5...3.16)

add_executable(map-unittest map_unittest.cpp)
target_include_directories(map-unittest PRIVATE ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(map-unittest
    PRIVATE
    SE::Map
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
add_test(NAME map-unittest COMMAND map-unittest)
