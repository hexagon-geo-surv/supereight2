cmake_minimum_required(VERSION 3.1...3.16)

set(CMAKE_THREAD_PREFER_PTHREAD ON)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
find_package(Eigen3)

add_executable(map-unittest map_unittest.cpp)
target_include_directories(map-unittest PRIVATE ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(map-unittest
    PRIVATE
    SE::Map
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
add_test(NAME map-unittest COMMAND map-unittest)