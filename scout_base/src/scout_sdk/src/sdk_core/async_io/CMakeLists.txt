# Dependency libraries
# find_package(LIB_NAME REQUIRED)

# Add libraries
set(ASYNC_IO_LIB_SRC
    src/async_serial.cpp
    src/async_can.cpp
)
add_library(asyncio STATIC ${ASYNC_IO_LIB_SRC})
set_target_properties(asyncio PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(asyncio PUBLIC "-DASIO_ENABLE_OLD_SERVICES -DASIO_HAS_POSIX_STREAM_DESCRIPTOR")
target_link_libraries(asyncio asio pthread)
target_include_directories(asyncio PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()

# # 'make install' to the correct locations (provided by GNUInstallDirs).
# install(TARGETS lib EXPORT MyLibraryConfig
# ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
# LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
# RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
# install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# # This makes the project importable from the install directory
# # Put config file in per-project dir (name MUST match), can also
# # just go into 'cmake'.
# install(EXPORT MyLibraryConfig DESTINATION share/MyLibrary/cmake)

# # This makes the project importable from the build directory
# export(TARGETS lib FILE MyLibraryConfig.cmake)