# Safe Bayesian Optimization

ROS 2 package for safe subgoal selection and reactive navigation using Bayesian optimization, terrain uncertainty, and polygon-based obstacle avoidance.

## What It Does

This package builds a safe navigation loop for scouting robots:

1. Requests terrain and uncertainty maps from the mapping stack.
2. Computes safe and informative candidate subgoals with Bayesian optimization.
3. Converts unsafe terrain into polygonal obstacles and safe-region boundaries.
4. Publishes subgoals to a reactive planner.
5. Drives a simulated robot with diffeomorphism-based obstacle avoidance.
6. Publishes markers and optional debug images for Foxglove/RViz-style visualization.

## Main Components

| Executable | Purpose |
| --- | --- |
| `safe_bayesian_optimization_node` | Computes safe subgoals from terrain data, uncertainty, and goal preferences. |
| `reactive_navigation_node` | Tracks subgoals while avoiding polygonal obstacles. |
| `goal_point_publisher` | Publishes test or user-selected goal points. |
| `turtlesim_spatial_publisher.py` | Publishes simulated spatial data for turtlesim-based testing. |
| `evaluation_node.py` | Helper script for evaluation workflows. |

## Features

- Bayesian optimization for safe and informative subgoal selection
- Terrain uncertainty-aware safety constraints
- Goal, frontier, and expansion scoring for subgoal ranking
- Diffeomorphism-based reactive navigation around polygonal obstacles
- Concave hull / alpha-shape safe-region processing with CGAL
- Voronoi/free-space support through the reactive planner geometry library
- ROS 2-native topics, services, messages, launch files, and parameters
- Foxglove-compatible visualization markers and optional debug images

## Requirements

### ROS 2

- ROS 2 Humble
- `ament_cmake`
- C++14-compatible compiler
- CMake 3.8+

### ROS Packages

Core package dependencies include:

- `rclcpp`, `rclpy`
- `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `cv_bridge`
- `turtlesim`
- `trusses_custom_interfaces`

The full demo launch also expects external packages such as:

- `mapping_package`
- `mapping_collector`
- `foxglove_visualization`
- `foxglove_bridge`

### System Libraries

- Eigen3
- OpenCV
- Boost
- CGAL
- Qhull
- Qt5 Core/Widgets
- libcurl
- PCL development libraries
- Python packages: `numpy`, `shapely`

## Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select safe_bayesian_optimization --cmake-args -DBUILD_EXAMPLES=OFF
source install/setup.bash
```

Or build the full workspace:

```bash
colcon build --cmake-args -DBUILD_EXAMPLES=OFF
source install/setup.bash
```

## Run

### Turtlesim / local scouting demo

```bash
ros2 launch safe_bayesian_optimization safe_scout.launch.py
```

This launches:

- safe Bayesian optimization node
- goal point publisher
- reactive navigation node
- turtlesim
- turtlesim spatial publisher

### Full mapping and visualization demo

```bash
ros2 launch safe_bayesian_optimization safe_bayesian_optimization.launch.py
```

This includes the local demo nodes plus mapping, data collection, Foxglove visualization, and Foxglove bridge nodes.

### Override config paths

```bash
ros2 launch safe_bayesian_optimization safe_scout.launch.py \
  config_file:=/path/to/safe_bayesian_optimization.yaml \
  reactive_planner_config:=/path/to/reactive_planner.yaml
```

## Configuration

Configuration files are in [`config/`](config/).

### `config/safe_bayesian_optimization.yaml`

Key parameter groups:

- `opt.beta`: exploration weight for Bayesian optimization
- `opt.lipshitz`: Lipschitz safety parameter
- `opt.f_max`: maximum accepted terrain/function value
- `opt.subgoal_erosion`: erosion margin for safe subgoal generation
- `opt.preference_order`: ordered scoring preferences, e.g. goal/frontier/expansion
- `opt.update_on_measurement`: update safe region on new measurements vs. on subgoal completion
- `opt.selection_strategy`: subgoal selection mode, e.g. `max` or `alternating_preference`
- `terrain_map.*`: map dimensions and resolution
- `debug.publish_debug_image`: enable/disable debug image publishing
- `robot.pose_topic`: robot pose topic used by the optimizer
- `node.update_rate`: optimizer update rate in Hz
- `visualization.topic_whitelist`: marker topics for visualization tooling

### `config/reactive_planner.yaml`

Key parameters:

- `p`: R-function exponent
- `epsilon`: switch distance
- `varepsilon`: polygon dilation allowance
- `mu_1`, `mu_2`: switch scaling exponents
- `robot_radius`: collision radius in meters
- `linear_gain`, `angular_gain`: controller gains
- `linear_cmd_limit`, `angular_cmd_limit`: velocity limits
- `goal_tolerance`: stopping tolerance near the current subgoal
- `next_subgoal_tolerance`: distance threshold for subgoal completion
- `naive_mode`: bypass diffeomorphism and use direct proportional control

### `config/lpsc.yaml`

Parameters for the larger LPSC/mapping demo, including visualization ranges, data collection settings, and terrain mapping parameters.

## Interfaces

### Custom Message

| Message | Description |
| --- | --- |
| `safe_bayesian_optimization/PolygonArray` | Array of `geometry_msgs/Polygon` obstacles. |

### Published Topics

#### `safe_bayesian_optimization_node`

| Topic | Type | Description |
| --- | --- | --- |
| `/current_subgoal` | `geometry_msgs/Point` | Current safe subgoal for the reactive planner. |
| `/subgoal_marker` | `visualization_msgs/Marker` | Current subgoal visualization. |
| `/polygon_array` | `safe_bayesian_optimization/PolygonArray` | Obstacle polygons from unsafe terrain. |
| `/envelope_polygon` | `geometry_msgs/Polygon` | Workspace envelope boundary. |
| `/concave_markers` | `visualization_msgs/MarkerArray` | Safe-region boundary visualization. |
| `/debug_polygons_image` | `sensor_msgs/Image` | Optional debug image. |

#### `reactive_navigation_node`

| Topic | Type | Description |
| --- | --- | --- |
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | Velocity command output. |
| `/freespace_markers` | `visualization_msgs/MarkerArray` | Local free-space visualization. |
| `/envelope_markers` | `visualization_msgs/MarkerArray` | Workspace envelope visualization. |

#### `goal_point_publisher`

| Topic | Type | Description |
| --- | --- | --- |
| `/goal_point` | `geometry_msgs/Point` | Test or user goal point. |
| `/goal_marker` | `visualization_msgs/Marker` | Goal point visualization. |

### Subscribed Topics

#### `safe_bayesian_optimization_node`

| Topic | Type | Description |
| --- | --- | --- |
| `/goal_point` | `geometry_msgs/Point` | Target goal for planning. |

#### `reactive_navigation_node`

| Topic | Type | Description |
| --- | --- | --- |
| `/polygon_array` | `safe_bayesian_optimization/PolygonArray` | Obstacle polygons. |
| `/envelope_polygon` | `geometry_msgs/Polygon` | Workspace envelope. |
| `/turtle1/pose` | `turtlesim/Pose` | Simulated robot pose. |
| `/current_subgoal` | `geometry_msgs/Point` | Optimizer-selected subgoal. |

### Service Clients

#### `safe_bayesian_optimization_node`

| Service | Type | Description |
| --- | --- | --- |
| `get_spatial_data` | `trusses_custom_interfaces/SpatialData` | Requests spatial measurements for terrain analysis. |
| `get_terrain_map_with_uncertainty` | `trusses_custom_interfaces/GetTerrainMapWithUncertainty` | Requests terrain map and uncertainty estimates. |

## Repository Layout

```text
config/      Runtime parameter YAML files
include/     Public C++ headers
launch/      ROS 2 launch files
msg/         Custom ROS message definitions
src/         C++ nodes, planner libraries, and Python helper nodes
data/        Package data assets
```

## License

Apache-2.0
