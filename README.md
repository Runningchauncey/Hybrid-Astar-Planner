# Hybrid-Astar-Planner

This project hosts trajectory planner based on Hybrid A\* for unstructured environment

## Documentation

This project consists of several ROS packages, which are:

- Part 1: trajectory planner
  - `mpros_planner`: node for trajectory planner
  - `mpros_algorithm`: path planner with Hybrid A\* algorithm
  - `mpros_msgs`: trajectory messages for communication between planner and controller
  - `mpros_navigation_map`: preprocessing block, generating different map representations
  - `mpros_rs_path`: Reeds-Shepp path generator
  - `mpros_smoother`: path optimizer with Ceres
  - `mpros_speed_profile`: speed profile generator according to S-Curve
  - `mpros_utils`: utils functions for planner

## Dependencies

- ROS Noetic <br>
  see [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Eigen3, version >= 3.4
  - Download source code from [Eigen Releases](https://gitlab.com/libeigen/eigen/-/releases)
  - Extract compressed file
  - Find the path to local eigen like `/usr/include/eigen3`
  - Install new Eigen lib by replacing old version
  ```bash
  cd eigen-3.4.0
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr # to replace eigen 3 in /usr/include/eigen3
  ```
  - check installation by
  ```c++
  std::cout << EIGEN_WORLD_VERSION << "."
            << EIGEN_MAJOR_VERSION << "."
            << EIGEN_MINOR_VERSION << std::endl;
  ```
- OMPL
  - see [OMPL Installation](https://ompl.kavrakilab.org/installation.html), recommend installation with ROS as shown below
  - Installation with ROS as follows
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-ompl
  ```
- CERES
  - follow the [Ceres Installation](http://ceres-solver.org/installation.html)

## Usage - To be tested due to removed simulation environment

- Clone all mpros package to your workspace which is shown below as `catkin_ws`:
  ```bash
  cd ~/catkin_ws/src
  # clone this repo to local
  git clone git@github.com:Runningchauncey/Hybrid-Astar-Planner.git
  # make the workspace
  cd ~/catkin_ws
  source /opt/ros/noetic/setup.bash
  catkin_make
  source devel/setup.bash
  # launch the simulation environment
  roslaunch mpros_planner vehicle_test.launch case:=0
  # run planner node in another terminal
  source devel/setup.bash
  rosrun mpros_planner main_planner
  ```
- Set up initial pose and goal pose in Rviz with `2D Pose Estimate` and `2D Nav Goal` by press the button, click a position and drag the arrow to set orientation

## Configure

- The packages `mpros_algorithm`, `mpros_navigation_map`, `mpros_rs_path`, `mpros_smoother`, `mpros_speed_profile` are configurable by load parameter file in `.yaml` format
- Parameter explanation see README in each package
- Configuration files under `mpros_planner/config`
- Load a configuration file while simulation environment on by
  ```bash
  rosparam load /path/to/file
  ```

## Test cases

All test cases can be load by change the case number when launch simulation environment.

- 0: reverse parking
- 1: parallel parking
- 2: Arena part
- 3: Arena whole map
- 4: slalom map
- 5: parking lot

## TODO

- [ ] Custom simulation environment
- [ ] Dubins Curve generation
