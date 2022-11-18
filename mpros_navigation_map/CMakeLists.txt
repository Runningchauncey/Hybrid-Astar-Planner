cmake_minimum_required(VERSION 3.0.2)
project(mpros_navigation_map)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  tf2
  roscpp
)
find_package(Eigen3 3.4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpros_navigation_map
  CATKIN_DEPENDS nav_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/nav_map.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

