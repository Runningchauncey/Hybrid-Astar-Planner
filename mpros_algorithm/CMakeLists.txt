cmake_minimum_required(VERSION 3.0.2)
project(mpros_algorithm)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  mpros_navigation_map
  mpros_rs_path
  mpros_utils
  roscpp
)
find_package(Eigen3 3.4 REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES hybrid_a_star
 CATKIN_DEPENDS mpros_navigation_map mpros_rs_path mpros_utils roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

add_library(hybrid_a_star
  src/hybrid_a_star.cpp
)
target_link_libraries(hybrid_a_star ${catkin_LIBRARIES} ${OMPL_LIBRARIES})


install(TARGETS hybrid_a_star
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
