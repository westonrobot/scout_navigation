# Dependency libraries
# find_package(LIB_NAME REQUIRED)

# Add libraries
set(ASYNC_IO_LIB_SRC
    src/async_serial.cpp
    src/async_can.cpp
)
add_library(asyncio STATIC ${ASYNC_IO_LIB_SRC})
target_link_libraries(asyncio asio pthread)
target_compile_definitions(asyncio PUBLIC "-DASIO_ENABLE_OLD_SERVICES -DASIO_HAS_POSIX_STREAM_DESCRIPTOR")
target_include_directories(asyncio PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()