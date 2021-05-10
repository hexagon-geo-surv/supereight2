cmake_minimum_required(VERSION 3.5...3.16)

add_executable(keyops-unittest keyops_unittest.cpp)
target_include_directories(keyops-unittest PRIVATE ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(keyops-unittest
    PRIVATE
    SE::Map
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
add_test(NAME keyops-unittest COMMAND keyops-unittest)
