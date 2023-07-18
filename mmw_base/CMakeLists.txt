cmake_minimum_required(VERSION 2.8.3)
project(mmw_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roslint 
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    mmw_msgs
    mmw_sdk
    tf2 
    tf2_ros
)
    
# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mmw_messenger
  CATKIN_DEPENDS mmw_msgs roscpp sensor_msgs mmw_sdk
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

add_library(mmw_messenger STATIC src/mmw_messenger.cpp)
target_link_libraries(mmw_messenger ${catkin_LIBRARIES})
set_property(TARGET mmw_messenger PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(mmw_base_node src/mmw_base_node.cpp)
target_link_libraries(mmw_base_node mmw_messenger ${catkin_LIBRARIES})


#############
## Install ##
#############

roslaunch_add_file_check(launch)

install(TARGETS mmw_messenger mmw_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
