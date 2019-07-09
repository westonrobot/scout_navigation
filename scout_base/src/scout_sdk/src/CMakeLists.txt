# Add source directories
add_subdirectory(demo)
add_subdirectory(sdk_core)
add_subdirectory(third_party)

if(BUILD_MONITOR)
add_subdirectory(monitor)
endif()