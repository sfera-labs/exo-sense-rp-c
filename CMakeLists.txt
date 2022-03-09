cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(exo_sense_rp)

add_subdirectory(lib/exo_sense_rp)
add_subdirectory(example)
