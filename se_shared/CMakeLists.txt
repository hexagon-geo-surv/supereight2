cmake_minimum_required(VERSION 3.5...3.16)

# Create library ##############################################################
set(LIB_NAME "se-shared")
add_library(${LIB_NAME} STATIC
    "src/image_utils.cpp"
    "src/sensor.cpp"
    "src/str_utils.cpp"
    "src/perfstats.cpp"
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_NAME}
  PUBLIC
    Lode::PNG
    SRL::Projection
)
set_target_properties(${LIB_NAME} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

# Add an alias so that the library can be used inside the build tree, e.g. when
# testing
add_library(SE::Shared ALIAS ${LIB_NAME})
# This is required so that the exported target has the name Shared and not
# se-shared
set_target_properties(${LIB_NAME} PROPERTIES EXPORT_NAME Shared)

