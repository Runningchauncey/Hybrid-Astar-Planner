# MPROS Navigation Map
This package hosts map representations derived from received occupancy grid, includes goal cost map, Voronoi Field, inflated map, and downsampled map.

## Dependencies

### External packages
- Eigen3, version >= 3.4

### ROS packages
- tf2
- nav_msgs

## Feature
- Downsample occupancy grid with given downsampling factor
- Goal cost map based on downsampled map, indicating holonomic distance to target position considering obstacles
- Voronoi Field builds potential field regarding distance to obstacles and clearance among obstacles
- Inflated map inflate original map with given radius for collision checking

## Parameters
```yaml
mpros:
    # src/mpros/mpros_planner/config/planner_config.yaml
    map:
        downsampling_factor: 2 # map downsampling factor, suggest to set so as downsampled grid size ~ 1
        obstacle_inflation_radius: 1.0 # inflation radius of obstacles, no smaller than half of vehicle width

```