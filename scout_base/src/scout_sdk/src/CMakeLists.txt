# Add source directories
add_subdirectory(apps)
add_subdirectory(comm)
add_subdirectory(scout_base)
add_subdirectory(third_party)

if(BUILD_TESTS)
    add_subdirectory(unit_tests)
endif()
