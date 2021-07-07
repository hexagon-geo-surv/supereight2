cmake_minimum_required(VERSION 3.5...3.16)

add_executable(str-utils-unittest "str_utils_unittest.cpp")
target_link_libraries(str-utils-unittest PRIVATE ${GTEST_MAIN_LIBRARIES})
add_test(NAME str-utils-unittest COMMAND str-utils-unittest)

