cmake_minimum_required(VERSION 2.8.3)
project(scarab_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  gazebo_ros
  gazebo_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

#include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

# Build whatever you need here
#add_library(...) # TODO
add_library(${PROJECT_NAME} src/cost_plugin.cpp)

catkin_package(
    DEPENDS
      gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

add_executable("${PROJECT_NAME}_bin" src/cost_plugin.cpp)

target_link_libraries("${PROJECT_NAME}_bin" ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# now we rename scarab_gazebo-bin executable to scarab_gazebo using target properties
set_target_properties("${PROJECT_NAME}_bin"
        PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")