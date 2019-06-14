cmake_minimum_required(VERSION 3.0.0)
project(driver)

## Put all binary files into /bin and libraries into /lib
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Add sub source directories
add_subdirectory(async_io)
