cmake_minimum_required(VERSION 3.5...3.16)

find_package(GLUT)
find_package(OpenGL)
# Main executable #########################################################
set(EXE_NAME main_tsdf)

set(READER_SRC
    src/reader_base.cpp
    src/reader_iclnuim.cpp
    src/reader_newercollege.cpp
    src/reader_openni.cpp
    src/reader_raw.cpp
    src/reader.cpp)

add_executable(${EXE_NAME}  "src/main_tsdf.cpp" ${READER_SRC})
target_include_directories(${EXE_NAME} BEFORE PRIVATE include)
target_link_libraries(${EXE_NAME}
  PRIVATE
    SE::Tracker
    stdc++fs
)

# Compile with GUI support
if(GLUT_FOUND)
    target_link_libraries(${EXE_NAME}
            PRIVATE
            ${GLUT_LIBRARIES}
            ${OPENGL_LIBRARIES}
            )
    target_compile_definitions(${EXE_NAME} PRIVATE "-DSE_GLUT")
    if(NOT APPLE)
        target_link_libraries(${EXE_NAME} PRIVATE GL)
    endif()
endif()

# Allow handling large files in 32-bit systems
target_compile_definitions(${EXE_NAME} PRIVATE "-D_FILE_OFFSET_BITS=64")
set_target_properties(${EXE_NAME} PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

