# MPROS Speed Profiler
This package generates speed profile for a path according to its geometric information

## Dependencies
### Internal
- mpros_utils

## Features
- Divide path into sections separated by cusp points
- Section into segments by curvature value
- Fit S-Curve to each segment to form a smooth speed profile
- Interpolate trajectory points by Bezier curve

## Parameters
```yaml
mpros:
  # src/mpros/mpros_planner/config/planner_config.yaml
  profile:
    debug: false
    jerk_limit: 0.5 # maximum jerk in [m/s/s/s]
    longitudinal_acc_limit: 1.0 # maximum longitudinal acceleration in [m/s/s]
    lateral_acc_limit: 0.5 # maximum lateral acceleration in [m/s/s]
    velocity_limit: 5.0 # maximum velocity in [m/s]
    time_step: 0.1 # time step to interpolate trajectory
    start_time: 0.0 # time at start point
    start_speed: 0.0 # velocity at the start point
    end_speed: 0.0 # velocity at the goal point
```