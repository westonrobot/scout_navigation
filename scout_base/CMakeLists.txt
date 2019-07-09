cmake_minimum_required(VERSION 2.8.3)
project(scout_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    controller_manager 
    hardware_interface 
    roslaunch 
    roslint 
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    scout_msgs
    tf)
    
# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scout_base
  CATKIN_DEPENDS hardware_interface scout_msgs roscpp sensor_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# add scout sdk
add_subdirectory(src/scout_sdk)

add_library(scout_messenger STATIC src/scout_messenger.cpp)
target_link_libraries(scout_messenger scoutbase ${catkin_LIBRARIES})
set_property(TARGET scout_messenger PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(scout_base_node src/scout_base_node.cpp)
target_link_libraries(scout_base_node scoutbase scout_messenger ${catkin_LIBRARIES})

#############
## Install ##
#############

# roslaunch_add_file_check(launch)

# install(TARGETS scoutcpp scoutio scout_base
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
