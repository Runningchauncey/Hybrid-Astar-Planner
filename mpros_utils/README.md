# MPROS Utils
This package hosts headers of utils functions.

## Features
- `footprint.h` generates footprint kernels of all discrete heading angles for collision checking
- `line_iterator.h` defines a iterator for straight line segment on grid map
- `pathpoint.h` defines a basic data structure for path
- `planner_utils.h` hosts functions like calculating curvature, calculating orientation, slicing vector, Bezier functions.
- `rs_statespace.h` utilizes OMPL::RSStateSpace to calculate RS path length as heuristic in planner
- `visualizer.h` publishes data as visual messages