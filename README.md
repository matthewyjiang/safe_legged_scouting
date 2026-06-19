# Safe Bayesian Optimization

`safe_bayesian_optimization` is a ROS 2 package for safe subgoal selection and reactive navigation. It combines Bayesian optimization, terrain uncertainty, and polygon-based obstacle avoidance to help a scouting robot move toward useful goals while staying in safe regions.

## Overview

The navigation loop is:

1. Request a terrain map with uncertainty from the mapping stack.
2. Score candidate subgoals for safety, goal progress, frontier value, and expansion value.
3. Convert unsafe terrain into polygonal obstacles and safe-region boundaries.
4. Publish the selected safe subgoal.
5. Track the subgoal with a reactive planner.
6. Publish visualization markers for Foxglove or RViz-style debugging.

## Package Contents

| Executable | Description |
| --- | --- |
| `safe_bayesian_optimization_node` | Selects safe, informative subgoals from terrain and uncertainty data. |
| `reactive_navigation_node` | Tracks subgoals while avoiding polygonal obstacles. |
| `goal_point_publisher` | Publishes configured, user-provided, or exploration goals. |
| `turtlesim_spatial_publisher.py` | Publishes simple simulated spatial updates for local testing. |
| `evaluation_node.py` | Helper node for evaluation workflows. |

## Highlights

- Safe Bayesian optimization over uncertain terrain
- Goal, frontier, and expansion-based subgoal scoring
- `max` and `alternating_preference` subgoal selection strategies
- Diffeomorphism-based reactive navigation around polygon obstacles
- Optional naive proportional-control mode for debugging
- CGAL-based safe-region and polygon processing
- ROS-native launch files, parameters, messages, services, and visualization topics

## Requirements

### ROS 2

- ROS 2 Humble
- `ament_cmake`
- CMake 3.8+
- C++14-compatible compiler

### Package Dependencies

Core ROS dependencies:

- `rclcpp`, `rclpy`
- `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`, `std_msgs`, `std_srvs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `cv_bridge`, `turtlesim`
- `trusses_custom_interfaces`
- `safe_scout_simulator`

System/library dependencies:

- Eigen3, OpenCV, Boost System, CGAL Core, Qhull
- Qt5 Core/Widgets, libcurl, PCL development libraries
- Python: `numpy`, `shapely`

The full mapping demo also expects:

- `mapping_package`
- `mapping_collector`
- `foxglove_visualization`
- `foxglove_bridge`

## Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select safe_bayesian_optimization --cmake-args -DBUILD_EXAMPLES=OFF
source install/setup.bash
```

To build the full workspace instead:

```bash
colcon build --cmake-args -DBUILD_EXAMPLES=OFF
source install/setup.bash
```

## Test

After building the package, run the lint and characterization tests:

```bash
colcon test --packages-select safe_bayesian_optimization
colcon test-result --verbose
```

This includes the subgoal preference characterization test.

## Run

### Local scouting demo

```bash
ros2 launch safe_bayesian_optimization safe_scout.launch.py
```

Launches the optimizer, goal publisher, reactive planner, turtlesim, and local spatial-data publisher.

### Full mapping and visualization demo

```bash
ros2 launch safe_bayesian_optimization safe_bayesian_optimization.launch.py
```

Launches the local demo plus mapping, data collection, Foxglove visualization, and Foxglove bridge nodes.

### Custom config files

```bash
ros2 launch safe_bayesian_optimization safe_scout.launch.py \
  config_file:=/path/to/safe_bayesian_optimization.yaml \
  reactive_planner_config:=/path/to/reactive_planner.yaml
```

## Configuration

Runtime configs live in [`config/`](config/).

### Optimizer config

File: [`config/safe_bayesian_optimization.yaml`](config/safe_bayesian_optimization.yaml)

| Parameter | Description |
| --- | --- |
| `opt.beta` | Bayesian optimization exploration weight. |
| `opt.lipschitz` | Lipschitz safety parameter. |
| `opt.f_max` | Maximum accepted terrain/function value. |
| `opt.subgoal_erosion` | Safety erosion margin for subgoal generation. |
| `opt.preference_order` | Ordered scoring preferences, e.g. `g`, `l`, `e`. |
| `opt.selection_strategy` | Selection mode: `max` or `alternating_preference`. |
| `opt.update_on_measurement` | Update safe region on new measurements instead of only after subgoal completion. |
| `opt.expansion_score_k` | Expansion-score decay parameter. |
| `opt.goal_score_k` | Goal-distance score decay parameter. |
| `terrain_map.*` | Map dimensions, cell counts, and/or resolution for map requests. |
| `robot.pose_topic` | Robot pose topic used by the optimizer. |
| `debug.publish_debug_image` | Enables `/debug_polygons_image`. |
| `visualization.topic_whitelist` | Marker topics for visualization tooling. |

> Note: the node declares `opt.lipschitz`; older configs may contain the misspelled key `opt.lipshitz`.

### Reactive planner config

File: [`config/reactive_planner.yaml`](config/reactive_planner.yaml)

| Parameter | Description |
| --- | --- |
| `p` | R-function exponent. |
| `epsilon` | Switch distance. |
| `varepsilon` | Polygon dilation allowance. |
| `mu_1`, `mu_2` | Switch scaling exponents. |
| `robot_radius` | Collision radius in meters. |
| `linear_gain`, `angular_gain` | Controller gains. |
| `linear_cmd_limit`, `angular_cmd_limit` | Velocity limits. |
| `goal_tolerance` | Stop tolerance near the current subgoal. |
| `next_subgoal_tolerance` | Distance threshold for reporting subgoal completion. |
| `naive_mode` | Use direct proportional control instead of diffeomorphism-based navigation. |

### LPSC/mapping config

File: [`config/lpsc.yaml`](config/lpsc.yaml)

Used by the larger mapping demo for visualization ranges, data collection, terrain mapping, and waypoint settings.

## Interfaces

### Custom message

| Message | Description |
| --- | --- |
| `safe_bayesian_optimization/PolygonArray` | Array of `geometry_msgs/Polygon` obstacles. |

### Core topics

| Topic | Type | Direction | Description |
| --- | --- | --- | --- |
| `/goal_point` | `geometry_msgs/PointStamped` | Goal publisher → optimizer/planner | Current target goal. |
| `/current_subgoal` | `geometry_msgs/Point` | Optimizer → planner | Selected safe subgoal. |
| `/polygon_array` | `safe_bayesian_optimization/PolygonArray` | Optimizer → planner | Obstacle polygons from unsafe terrain. |
| `/envelope_polygon` | `geometry_msgs/Polygon` | Optimizer → planner | Workspace envelope boundary. |
| `/spirit/current_pose` | `geometry_msgs/Pose` | Robot/sim → nodes | Robot pose. Optimizer topic is configurable with `robot.pose_topic`. |
| `/spirit/planner_twist_command` | `geometry_msgs/Twist` | Planner → robot/sim | Velocity command. |
| `/subgoal_reached` | `std_msgs/Empty` | Planner → optimizer | Subgoal completion notification. |
| `/spatial_data_size` | `std_msgs/Int32` | Spatial publisher → optimizer | Measurement-count update trigger. |
| `/input_goal_point` | `geometry_msgs/PointStamped` | User/tool → goal publisher | Adds a runtime goal point. |

### Visualization topics

Common marker topics include:

- `/goal_marker`, `/goal_markers`
- `/subgoal_marker`, `/projected_goal_marker`
- `/concave_markers`, `/envelope_markers`
- `/freespace_markers`, `/local_workspace_markers`
- `/frontier_goal_score_markers`
- `/frontier_expansion_prob_markers`
- `/frontier_overall_score_markers`
- `/frontier_goal_distance_markers`
- `/frontier_robot_distance_markers`
- `/debug_polygons_image` when enabled

### Services

| Node | Service | Type | Description |
| --- | --- | --- | --- |
| `safe_bayesian_optimization_node` | `/get_terrain_map_with_uncertainty` | `trusses_custom_interfaces/GetTerrainMapWithUncertainty` | Requests terrain and uncertainty maps. |
| `goal_point_publisher` | `/increment_goal_index` | `std_srvs/Trigger` | Advances to the next configured waypoint. |
| `evaluation_node.py` | `/sample_points` | `trusses_custom_interfaces/SamplePoints` | Requests sample points for evaluation. |

## Repository Layout

```text
config/      Runtime parameter YAML files
include/     Public C++ headers and planner interfaces
launch/      ROS 2 launch files
msg/         Custom ROS message definitions
src/         C++ nodes, planner libraries, and Python helper nodes
data/        Package data assets
plans/       Development notes and implementation plans
```

## License

Apache-2.0
