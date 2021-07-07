cmake_minimum_required(VERSION 3.5...3.16)

set(READER_SRC
  ../../src/reader_base.cpp
  ../../src/reader_iclnuim.cpp
  ../../src/reader_newercollege.cpp
  ../../src/reader_openni.cpp
  ../../src/reader_raw.cpp
  ../../src/reader_tum.cpp
  ../../src/reader.cpp
  ../../src/config.cpp
)

add_executable(multires-ofusion-systemtest multires_ofusion_systemtest.cpp ${READER_SRC})
target_include_directories(multires-ofusion-systemtest PRIVATE ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ../../include)
target_link_libraries(multires-ofusion-systemtest
  PRIVATE
  SE::Map
  SE::Shared
  SE::Integrator
  SE::Tracker
  stdc++fs
  ${GTEST_BOTH_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
add_test(NAME multires-ofusion-systemtest COMMAND multires-ofusion-systemtest)
