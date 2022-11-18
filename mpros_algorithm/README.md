# MPROS Algorithm
This package hosts the path planning algorithm for trajectory planner, including Hybrid A* now.

## Dependencies

### External packages
- Eigen3, version >= 3.4
- OMPL

### Internal packages
- mpros_navigation_map
- mpros_rs_path
- mpros_utils

## Feature
- Realized continuous configuration transition in a discrete grid map
- Heuristic includes both holonomic and nonholonomic
- Path cost includes accumulated path length, maneuver penalties, and Voronoi Field value
- Four collision checking methods to be chosen from
- Analytical expansion by Reeds-Shepp path
- Interpolation by Bezier to smooth the path

## Parameters
```yaml
mpros:
    # mpros/mpros_planner/config/planner_config.yaml
    planner:
        optimal_path: false # if searching for optimal path after found the target
        interpolation_enable: true # if interpolate the path
        step_size: 2.0 # search step size, has to be a bit larger than downsampled grid size
        max_iteration: 100000 #
        gear_penalty: 10.0 # penalty on change direction
        reverse_penalty: 10.0 # penalty on reversely driving
        steering_penalty: 0.5 # penalty on any steering angle
        steering_change_penalty: 0.5 # penalty on changing steering angle
        analytic_solution_range: 10.0 # range to start to looking for analytic solution - RS path
        yaw_discretization: 36 # number of discrete yaw values
        path_point_distance: 0.1 # path point distance, relevant for generating path
        steering_discretization: 5 # number of steering angle, has to be odd number due to 0 steering
    
    # mpros/mpros_planner/config/vehicle_config.yaml
    vehicle:
        max_steering_angle: 0.35 # in rad
        max_curvature: 0.125 # 1 / min_radius [1/m]
        width: 2.0
        wheelbase: 3.0
        length: 4.2
        center_to_rotate_center_longitudinal: 0.0 # rotate center offset from vehicle shape center
```

## Reference
[1] D. Dolgov, S. Thrun, M. Montemerlo, and J. Diebel, ‘Practical Search Techniques in Path Planning for Autonomous Driving’, p. 6. [Link](https://www.aaai.org/Papers/Workshops/2008/WS-08-10/WS08-10-006.pdf)