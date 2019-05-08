# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(demo_scout_cpp demo_scout_cpp.cpp)
target_link_libraries(demo_scout_cpp scoutcpp stopwatch)
