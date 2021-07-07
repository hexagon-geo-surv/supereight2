cmake_minimum_required(VERSION 3.5...3.16)

find_package(GLUT)
find_package(OpenGL)

set(READER_SRC
        src/reader_base.cpp
        src/reader_iclnuim.cpp
        src/reader_newercollege.cpp
        src/reader_openni.cpp
        src/reader_raw.cpp
        src/reader_tum.cpp
        src/reader.cpp
        src/config.cpp
        )

# Compile the TSDF pipelines.
set(SE_RES "Single" "Multi")
set(SE_FLD "TSDF" "Occupancy")
foreach(SE_FLD ${SE_FLD})
  foreach(SE_RES ${SE_RES})

    if (SE_FLD MATCHES "Occupancy")
      set(MAIN_SRC "src/main_ofusion.cpp")
      if (SE_RES MATCHES "Single")
        continue()
      endif()
    else()
      set(MAIN_SRC "src/main_tsdf.cpp")
    endif()

    string(TOLOWER ${SE_FLD} SE_FLD_LC)
    string(TOLOWER ${SE_RES} SE_RES_LC)

    set(EXE_TSDF_NAME "main_${SE_FLD_LC}_${SE_RES_LC}")

    add_executable(${EXE_TSDF_NAME} ${MAIN_SRC} ${READER_SRC})
    target_include_directories(${EXE_TSDF_NAME} BEFORE PRIVATE include)
    target_link_libraries(${EXE_TSDF_NAME}
            PRIVATE
			SE::Supereight
			Lode::PNG
            stdc++fs
            )
    target_compile_definitions(${EXE_TSDF_NAME}
            PRIVATE
            SE_RES=se::Res::${SE_RES}
            # Allow handling large files in 32-bit systems
            _FILE_OFFSET_BITS=64
            )

    # Compile with GUI support
    if(GLUT_FOUND)
      target_link_libraries(${EXE_TSDF_NAME}
              PRIVATE
              ${GLUT_LIBRARIES}
              ${OPENGL_LIBRARIES}
              )
      target_compile_definitions(${EXE_TSDF_NAME} PRIVATE "-DSE_GLUT")
      if(NOT APPLE)
        target_link_libraries(${EXE_TSDF_NAME} PRIVATE GL)
      endif()
    endif()
  endforeach()
endforeach()

