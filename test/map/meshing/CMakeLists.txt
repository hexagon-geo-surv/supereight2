cmake_minimum_required(VERSION 3.5...3.16)

add_executable(meshing-unittest meshing_unittest.cpp)
target_link_libraries(meshing-unittest PRIVATE ${GTEST_MAIN_LIBRARIES})
add_test(NAME meshing-unittest COMMAND meshing-unittest)

