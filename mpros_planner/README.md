# MPROS Planner
This package hosts the main function for trajectory planner

## Dependencies
### External
- Eigen3 version >= 3.4

### ROS
- tf2
- geometry_msgs
- gazebo_msgs

### Internal
- mpros_msgs
- mpros_navigation_map
- mpros_algorithm
- mpros_utils
- mpros_speed_profile
- mpros_smoother
- vehicle_control
    - for simulation environment

## Features
- Trajectory planning pipeline: preprocessing, planner. postprocessing, profile generation
- Interactively selecting start pose and goal pose
- Plot by matplotlibcpp 

## Usage
- Start simulation environment
    ```bash
    roslaunch mpros_planner vehicle_test
    ```
- Run planner
    ```bash
    rosrun mpros_planner main_planner
    ```
