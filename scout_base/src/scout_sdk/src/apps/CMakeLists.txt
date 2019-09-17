# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(app_scout_demo scout_demo/scout_demo.cpp)
target_link_libraries(app_scout_demo scoutbase)

# add_executable(demo_scout_can demo_scout_can.cpp)
# target_link_libraries(demo_scout_can scoutbase)

# add_executable(demo_scout_serial demo_scout_serial.cpp)
# target_link_libraries(demo_scout_serial scoutbase)

if(BUILD_MONITOR)
    add_subdirectory(scout_monitor)
endif()