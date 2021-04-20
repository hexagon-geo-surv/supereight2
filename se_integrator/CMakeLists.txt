cmake_minimum_required(VERSION 3.5...3.16)

# Create library ##############################################################
set(LIB_NAME "se-integrator")
add_library(${LIB_NAME} INTERFACE)
target_include_directories(${LIB_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_NAME}
  INTERFACE
    SE::Map
)

# Add an alias so that the library can be used inside the build tree, e.g. when
# testing
add_library(SE::Integrator ALIAS ${LIB_NAME})
# This is required so that the exported target has the name Shared and not
# se-shared
set_target_properties(${LIB_NAME} PROPERTIES EXPORT_NAME Integrator)

