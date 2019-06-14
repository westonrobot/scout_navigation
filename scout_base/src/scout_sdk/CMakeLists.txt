cmake_minimum_required(VERSION 3.0.0)
project(scout_sdk)

# generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/devel)

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Chosse build type
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

## Optionally built modules: ON/OFF
set(BUILD_TESTS OFF)

## Add source directories
add_subdirectory(src)
