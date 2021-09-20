cmake_minimum_required(VERSION 3.5...3.16)

find_package(GLUT)
find_package(OpenGL)
find_package(OpenCV REQUIRED COMPONENTS highgui imgproc core features2d)

# Compile the reader as a library so that it isn't recompiled for each app.
set(LIB_NAME reader)
add_library(${LIB_NAME} STATIC
  "src/config.cpp"
  "src/reader.cpp"
  "src/reader_base.cpp"
  "src/reader_iclnuim.cpp"
  "src/reader_newercollege.cpp"
  "src/reader_openni.cpp"
  "src/reader_raw.cpp"
  "src/reader_tum.cpp"
  "src/reader_interiornet.cpp"
)
target_include_directories(${LIB_NAME} PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${LIB_NAME} PRIVATE SE::Supereight Lode::PNG stdc++fs ${OpenCV_LIBS})



# Compile the apps for each configuration in SE_APP_CONFIG.
set(SE_APP_CONFIG
  "TSDF Single PinholeCamera"
  "TSDF Multi PinholeCamera"
  "Occupancy Multi PinholeCamera"
)
foreach(APP_CONFIG ${SE_APP_CONFIG})
  # Replace spaces with semicolons to convert the string to a CMake list.
  string(REPLACE " " ";" APP_CONFIG ${APP_CONFIG})
  # Get the field, resolution and sensor type from the list.
  list(GET APP_CONFIG 0 FIELD)
  list(GET APP_CONFIG 1 RES)
  list(GET APP_CONFIG 2 SENSOR)
  # Convert to lower case.
  string(TOLOWER ${FIELD} FIELD_LC)
  string(TOLOWER ${RES} RES_LC)
  string(TOLOWER ${SENSOR} SENSOR_LC)

  # Add a custom command that replaces the field, resolution and sensor in
  # main.cpp using sed. The generated file is then used as the main of the
  # app.
  set(INPUT_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
  set(GENERATED_MAIN "${CMAKE_CURRENT_BINARY_DIR}/main_${FIELD_LC}_${RES_LC}_${SENSOR_LC}.cpp")
  add_custom_command(
    OUTPUT  ${GENERATED_MAIN}
    COMMAND sed -e "s/se::TSDF/se::${FIELD}/g" -e "s/se::Res::Single/se::Res::${RES}/g" -e "s/se::PinholeCamera/se::${SENSOR}/g" ${INPUT_MAIN} > ${GENERATED_MAIN}
    DEPENDS ${INPUT_MAIN}
  )

  # Compile the app using the generated file.
  set(EXE_NAME "supereight_${FIELD_LC}_${RES_LC}_${SENSOR_LC}")
  add_executable(${EXE_NAME} ${GENERATED_MAIN})
  target_include_directories(${EXE_NAME} BEFORE PRIVATE include)
  target_link_libraries(${EXE_NAME} PRIVATE SE::Supereight ${LIB_NAME})
  # Allow handling large files in 32-bit systems
  target_compile_definitions(${EXE_NAME} PRIVATE _FILE_OFFSET_BITS=64)

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
endforeach()

