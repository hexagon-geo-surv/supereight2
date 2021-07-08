cmake_minimum_required(VERSION 3.5...3.16)

set(READER_SRC
  ../../../app/src/reader_base.cpp
  ../../../app/src/reader_iclnuim.cpp
  ../../../app/src/reader_newercollege.cpp
  ../../../app/src/reader_openni.cpp
  ../../../app/src/reader_raw.cpp
  ../../../app/src/reader_tum.cpp
  ../../../app/src/reader.cpp
  ../../../app/src/config.cpp
)

get_filename_component(CONFIG_FILE "config.yaml" ABSOLUTE)

add_executable(multires-ofusion-systemtest multires_ofusion_systemtest.cpp ${READER_SRC})
target_include_directories(multires-ofusion-systemtest PRIVATE ../../../app/include)
target_link_libraries(multires-ofusion-systemtest PRIVATE Lode::PNG stdc++fs)
add_test(NAME multires-ofusion-systemtest COMMAND multires-ofusion-systemtest ${CONFIG_FILE})

