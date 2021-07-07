cmake_minimum_required(VERSION 3.5...3.16)

add_executable(keyops-unittest keyops_unittest.cpp)
target_link_libraries(keyops-unittest PRIVATE ${GTEST_MAIN_LIBRARIES})
add_test(NAME keyops-unittest COMMAND keyops-unittest)

