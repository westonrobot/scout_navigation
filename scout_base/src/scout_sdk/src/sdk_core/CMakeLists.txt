# Add source directories
add_subdirectory(async_io)
add_subdirectory(scout_base)

if(BUILD_TESTS)
    add_subdirectory(unit_tests)
endif()