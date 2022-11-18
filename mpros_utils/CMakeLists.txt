cmake_minimum_required(VERSION 3.0.2)
project(mpros_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
