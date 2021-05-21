cmake_minimum_required(VERSION 3.5...3.16)

find_package(GLUT)
find_package(OpenGL)

set(READER_SRC
    src/reader_base.cpp
    src/reader_iclnuim.cpp
    src/reader_newercollege.cpp
    src/reader_openni.cpp
    src/reader_raw.cpp
    src/reader.cpp)

set(EXE_TSDF_NAME  "main_tsdf")
set(EXE_MTSDF_NAME "main_multires_tsdf")

add_executable(${EXE_TSDF_NAME} "src/main_tsdf.cpp" ${READER_SRC})
target_include_directories(${EXE_TSDF_NAME} BEFORE PRIVATE include)
target_link_libraries(${EXE_TSDF_NAME}
  PRIVATE
    SE::Tracker
    stdc++fs
)

add_executable(${EXE_MTSDF_NAME}  "src/main_multires_tsdf.cpp" ${READER_SRC})
target_include_directories(${EXE_MTSDF_NAME} BEFORE PRIVATE include)
target_link_libraries(${EXE_MTSDF_NAME}
  PRIVATE
    SE::Tracker
    stdc++fs
)

# Compile with GUI support
if(GLUT_FOUND)
  target_link_libraries(${EXE_TSDF_NAME}
    PRIVATE
      ${GLUT_LIBRARIES}
      ${OPENGL_LIBRARIES}
  )
  target_compile_definitions(${EXE_TSDF_NAME} PRIVATE "-DSE_GLUT")
  target_link_libraries(${EXE_MTSDF_NAME}
    PRIVATE
      ${GLUT_LIBRARIES}
      ${OPENGL_LIBRARIES}
  )
  target_compile_definitions(${EXE_MTSDF_NAME} PRIVATE "-DSE_GLUT")
  if(NOT APPLE)
    target_link_libraries(${EXE_TSDF_NAME} PRIVATE GL)
    target_link_libraries(${EXE_MTSDF_NAME} PRIVATE GL)
  endif()
endif()

# Allow handling large files in 32-bit systems
target_compile_definitions(${EXE_TSDF_NAME} PRIVATE "-D_FILE_OFFSET_BITS=64")
target_compile_definitions(${EXE_MTSDF_NAME} PRIVATE "-D_FILE_OFFSET_BITS=64")

