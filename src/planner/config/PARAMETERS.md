# Configuration Parameters Documentation

This document describes all parameters used in the planner configuration file (`config.yaml`).

## Common Parameters

Shared parameters referenced across multiple nodes using YAML anchors.

| Parameter | Type | Description |
|-----------|------|-------------|
| `osm_path` | string | Path to OpenStreetMap file |
| `_x` | float | Origin X coordinate (longitude) |
| `_y` | float | Origin Y coordinate (latitude) |
| `pose_topic` | string | ROS topic for vehicle pose |
| `telemetry_topic` | string | ROS topic for vehicle telemetry data |
| `yolo_topic` | string | ROS topic for YOLO detections |
| `no_entry_topic` | string | ROS topic for no-entry zone IDs |
| `switch_to_centerline_topic` | string | ROS topic for controller switching |
| `speed_limit_topic` | string | ROS topic for speed limit updates |
| `cluster_topic` | string | ROS topic for obstacle clusters |

## Global Planner

High-level path planning node.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `origin_pose._x` | float | 40.64479964953 | Origin X coordinate |
| `origin_pose._y` | float | 28.18269962266 | Origin Y coordinate |
| `osm_path` | string | - | Path to OSM map file |
| `rviz_goal_topic` | string | "/move_base_simple/goal" | RViz goal topic |
| `station_name` | string | "durak" | Station identifier in OSM |
| `park_name` | string | "park" | Parking area identifier in OSM |
| `rviz_pose` | bool | false | Use RViz for goal pose selection |
| `goal_pose_topic` | string | "/global_planner/goal" | Topic to publish goal poses |
| `first_goal_id` | int | 657 | Initial waypoint ID in OSM |
| `parking_lot_entry_id` | int | 2450 | Parking entry waypoint ID |
| `is_first_day` | bool | true | First day mission mode |
| `stopping_at_station_tolerance` | float | 1.0 | Station stop tolerance (meters) |
| `stopping_at_park_tolerance` | float | 1.3 | Parking stop tolerance (meters) |
| `station_marker_topic` | string | "/station_marker" | Station visualization marker topic |

## Mission Planner

Mission execution and path management node.

| Parameter | Type | Description |
|-----------|------|-------------|
| `origin_pose._x` | float | Origin X coordinate |
| `origin_pose._y` | float | Origin Y coordinate |
| `osm_path` | string | Path to OSM map file |
| `pose_topic` | string | Vehicle pose input topic |
| `path_topic` | string | Path output topic |
| `goal_pose_topic` | string | Goal pose input topic |
| `stop_topic` | string | Stop distance command topic |
| `telemetry_topic` | string | Vehicle telemetry input topic |
| `yolo_topic` | string | Object detection input topic |
| `station_name` | string | Station identifier |
| `park_name` | string | Parking area identifier |
| `switch_to_centerline_topic` | string | Controller mode switch topic |
| `no_entry_topic` | string | No-entry zones topic |
| `cluster_topic` | string | Obstacle clusters input topic |
| `occupancy_grid_topic` | string | Occupancy grid input topic |

## Occupancy Grid

Dynamic obstacle detection and grid generation node.

### Grid Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `grid_resolution` | float | 0.3 | Grid cell size (meters) |
| `grid_size` | int | 150 | Grid dimensions (cells) |
| `origin_pose._x` | float | 40.64479964953 | Origin X coordinate |
| `origin_pose._y` | float | 28.18269962266 | Origin Y coordinate |
| `osm_path` | string | - | Path to OSM map file |

### Vehicle Dimensions

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_width` | float | 1.4 | Vehicle width (meters) |
| `vehicle_length` | float | 2.0 | Vehicle length (meters) |
| `safety_margin` | float | 0.5 | Additional safety clearance (meters) |

### Obstacle Detection

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `base_obstacle_radius` | float | 0.4 | Base obstacle size (meters) |
| `obstacle_core_radius` | float | 1.5 | Core obstacle inflation radius (meters) |
| `lateral_clearance_factor` | float | 1.1 | Lateral inflation multiplier |
| `association_distance` | float | 2.0 | Max distance for obstacle tracking (meters) |

### Prediction Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `prediction_time_min` | float | 0.3 | Minimum prediction horizon (seconds) |
| `prediction_time_max` | float | 0.7 | Maximum prediction horizon (seconds) |
| `max_prediction_distance` | float | 2.0 | Max forward prediction distance (meters) |
| `static_velocity_threshold` | float | 0.5 | Speed threshold for static detection (m/s) |

### Probabilistic Tracking

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `prob_hit` | float | 0.7 | Detection probability when obstacle exists |
| `prob_miss` | float | 0.3 | Miss probability when obstacle exists |
| `prob_threshold_render` | float | 0.4 | Minimum probability to render obstacle |
| `prob_threshold_delete` | float | 0.1 | Minimum probability to keep tracking |
| `prediction_decay_rate` | float | 0.95 | Probability decay during occlusion (per second) |

### Filtering

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `alpha_position` | float | 0.3 | Position EMA smoothing factor (0-1) |
| `alpha_velocity` | float | 0.5 | Velocity EMA smoothing factor (0-1) |

### Topics

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pose_topic` | string | "/pcl_pose" | Vehicle pose input |
| `occupancy_grid_topic` | string | "/occupancy_grid" | Grid output topic |
| `cluster_topic` | string | "/all_bottom_clusters" | Obstacle clusters input |
| `path_topic` | string | "/mission_planner/path" | Reference path input |

### Debug

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `debug_mode` | bool | false | Enable debug output |

## Trajectory Planner

Local trajectory optimization node.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `origin_pose._x` | float | 40.64479964953 | Origin X coordinate |
| `origin_pose._y` | float | 28.18269962266 | Origin Y coordinate |
| `osm_path` | string | - | Path to OSM map file |
| `pose_topic` | string | "/pcl_pose" | Vehicle pose input |
| `follow_centerline` | bool | true | Follow road centerline |
| `curvature_contributing_factor` | float | 3.0 | Curvature cost weight |
| `centerline_contributing_factor` | float | 7.0 | Centerline deviation cost weight |
| `clearance` | int | 0 | Minimum clearance from obstacles (cells) |
| `occupancy_grid_topic` | string | "/occupancy_grid" | Occupancy grid input |
| `path_topic` | string | "/mission_planner/path" | Reference path input |
| `trajectory_topic` | string | "/trajectory_planner/trajectory" | Trajectory output |
| `switch_controller_topic` | string | "/switch_controller" | Controller mode topic |

## Controller

Vehicle control and trajectory tracking node.

### Topics

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory_topic` | string | "/trajectory_planner/trajectory" | Trajectory input |
| `pose_topic` | string | "/pcl_pose" | Vehicle pose input |
| `autonomous_cmd_topic` | string | "/controller/vehicle_control" | Control commands output |
| `stop_topic` | string | "/mission_planner/stop_distance" | Stop distance input |
| `telemetry_topic` | string | "/vehicle/telemetry" | Vehicle telemetry input |
| `switch_controller_topic` | string | "/switch_controller" | Controller mode input |

### Control Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `base_lookahead_distance` | float | 6.0 | Base lookahead distance (meters) |
| `sensitivity` | float | 5.5 | Steering sensitivity |
| `max_speed` | float | 185.0 | Maximum velocity command |
| `min_speed` | float | 30.0 | Minimum velocity command |
| `max_acceleration` | float | 30.0 | Maximum acceleration |
| `max_steering_angle` | float | 90.0 | Maximum steering angle (degrees) |
| `min_lookahead_distance` | float | 3.8 | Minimum lookahead distance (meters) |
| `brake_distance` | float | 20.0 | Distance to start braking (meters) |
| `wheelbase` | float | 2.0 | Vehicle wheelbase (meters) |

### PID Control

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `Kp` | float | 2.0 | Proportional gain (velocity reference) |
| `Kd` | float | 0.3 | Derivative gain (velocity reference) |
| `Ts` | float | 0.01 | Sampling time (seconds) |
| `Kp_speed` | float | 2.0 | Proportional gain (speed control) |
| `Ki_speed` | float | 0.3 | Integral gain (speed control) |
| `Kd_speed` | float | 0.01 | Derivative gain (speed control) |

## Notes

- All distance measurements are in meters unless specified otherwise
- All time measurements are in seconds unless specified otherwise
- Coordinate system uses latitude/longitude with local origin transformation
- Probability values range from 0.0 to 1.0
- EMA (Exponential Moving Average) smoothing factors range from 0.0 (no smoothing) to 1.0 (maximum smoothing)