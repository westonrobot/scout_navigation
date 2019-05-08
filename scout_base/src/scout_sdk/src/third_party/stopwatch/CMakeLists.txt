cmake_minimum_required(VERSION 3.0.0)
project(stopwatch)

# ascent library
add_library(stopwatch INTERFACE)
target_include_directories(stopwatch INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)