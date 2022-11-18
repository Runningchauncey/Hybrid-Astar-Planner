cmake_minimum_required(VERSION 3.0.2)
project(mpros_msgs)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Bahn.msg
  Waypoint.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
