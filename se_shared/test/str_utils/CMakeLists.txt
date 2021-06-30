cmake_minimum_required(VERSION 3.5...3.16)

add_executable(str-utils-unittest "str_utils_unittest.cpp")
target_include_directories(str-utils-unittest PRIVATE ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(str-utils-unittest
  PRIVATE
  SE::Map
  SE::Shared
  ${GTEST_BOTH_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
add_test(NAME str-utils-unittest COMMAND str-utils-unittest)
