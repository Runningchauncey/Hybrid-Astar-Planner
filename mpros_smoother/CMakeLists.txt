cmake_minimum_required(VERSION 3.0.2)
project(mpros_smoother)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf2
  mpros_navigation_map
  mpros_utils
)

find_package(Eigen3 3.4 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mpros_smoother
 CATKIN_DEPENDS nav_msgs roscpp std_msgs tf2 mpros_navigation_map mpros_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/smoother_wrapper.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

