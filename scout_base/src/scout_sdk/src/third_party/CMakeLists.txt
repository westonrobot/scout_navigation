## Add sub source directories
add_subdirectory(asio)

if(BUILD_TESTS)
add_subdirectory(googletest)
endif()