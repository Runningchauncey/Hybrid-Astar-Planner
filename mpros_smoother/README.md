# MPROS Smoother
This package uses ceres solver to optimize the given path.

## Dependencies
### External
- Eigen3, version >= 3.4
- CERES

### ROS
- std_msgs
- nav_msgs
- tf2

### Internal
- mpros_utils

## Features
- Optimize path with Ceres solver
- Cost function consists of smoothness, curvature and distance term
- Re-calculate heading angle and curvature after optimization 
- First two and last two points stay constant

## Parameters
```yaml
mpros:
  # src/mpros/mpros_planner/config/smoother_config.yaml
  smoother:
    enable_smoother: true # enable the smoother
    smooth_weight: 100 # cost weight for smoothness term
    curv_weight: 10.0 # cost weight for curvature term
    dist_weight: 0.1 # cost weight for distance term
    max_solver_time_in_seconds: 1.0 # max solver time
  optimizer:
    debug: true
    linear_solver: "SPARSE_NORMAL_CHOLESKY"
    minimizer: "LINE_SEARCH"
    max_iterations: 500
    parameter_tolerance: 1.0e-15
    function_tolerance: 1.0e-7
    gradient_tolerance: 1.0e-10
```