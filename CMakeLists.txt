cmake_minimum_required(VERSION 2.8.3)
project(scarab_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  gazebo_ros
  message_generation
)

add_service_files(DIRECTORY srv FILES getZ.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Build whatever you need here
add_library(${PROJECT_NAME} src/cost_plugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
    DEPENDS
      roscpp
      gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)
