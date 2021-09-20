cmake_minimum_required(VERSION 3.5...3.16)

# Disable warnings for third-party code
add_compile_options(-w)

# SRL Projection
option(SRL_PROJECTION_BUILD_TESTS "Enable testing" OFF)
add_subdirectory(srl_projection)

