cmake_minimum_required(VERSION 3.0.0)
project(scout_sdk)

# generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/devel)

## Chosse build type
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

## Optionally built modules: ON/OFF
set(BUILD_TESTS OFF)

## Add source directories
add_subdirectory(src)
