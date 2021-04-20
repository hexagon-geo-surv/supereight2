cmake_minimum_required(VERSION 3.1...3.16)

set(CMAKE_THREAD_PREFER_PTHREAD ON)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
find_package(Eigen3)

add_executable(keyops-unittest keyops_unittest.cpp)
target_include_directories(keyops-unittest PRIVATE ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(keyops-unittest
    PRIVATE
    SE::Map
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
add_test(NAME keyops-unittest COMMAND keyops-unittest)