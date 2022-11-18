cmake_minimum_required(VERSION 3.0.2)
project(mpros_planner)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  geometry_msgs
  gazebo_msgs
  tf2
  mpros_navigation_map
  mpros_smoother
  mpros_utils
  mpros_speed_profile
  mpros_rs_path
  mpros_algorithm
  mpros_msgs
)

find_package(Eigen3 3.4 REQUIRED)
find_package(Python REQUIRED COMPONENTS Development NumPy)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

add_executable(main_planner src/main_planner.cpp)
target_include_directories(main_planner PRIVATE ${Python_INCLUDE_DIRS} ${Python_NumPy_INCLUDE_DIRS}) 
target_link_libraries(main_planner ${catkin_LIBRARIES} Python::Python Python::NumPy)

add_executable(planner src/planner_ros.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})

