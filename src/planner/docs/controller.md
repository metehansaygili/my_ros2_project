# controller_node

## 🎯 Purpose
The `controller_node` implements a Pure Pursuit–style lateral controller and a simple longitudinal regulator. It consumes a planned trajectory, the current vehicle pose, stop‐distance commands, and telemetry, then publishes throttle, brake, steering, and related control commands for the vehicle.

---

## 🔢 Algorithm
1. **Subscribe** to:
   - A planned trajectory (`nav_msgs/Path`)  
   - Vehicle pose (`PoseWithCovarianceStamped`)  
   - Distance‐to‐stop (`std_msgs/Float64`)  
   - Vehicle telemetry (`gae_msgs/GaeTelemetry`)  
2. **On poseCallback**:
   - Find the closest point on the trajectory  
   - Compute a look‐ahead point based on current speed & `base_lookahead_distance`  
   - **Pure Pursuit**:  
     - Compute steering angle from look‐ahead geometry & `wheelbase`  
     - Apply `sensitivity` scaling & map into vehicle steering range  
   - **Longitudinal control**:  
     - Enforce `max_acceleration` limits on the desired speed  
     - Apply braking logic if `distance_to_stop < brake_distance`  
3. **Publish**:
   - A `GaeControlCmd` with steering, throttle/brake, gear, etc.  
   - A visualization marker at the look‐ahead point  

---

## ⚙️ Inner Logic
- **Constructor**  
  - Declare & read ROS2 parameters  
  - Create subscribers & publishers  
  - Initialize internal state (speeds, timers, look‐ahead)  
- **Callbacks**  
  - `trajectoryCallback(Path)`: cache the latest trajectory  
  - `distanceToStopCallback(Float64)`: update `distance_to_stop`  
  - `telemetryCallback(GaeTelemetry)`: update `current_speed` & timestamp  
  - `poseCallback(PoseWithCovarianceStamped)`:  
    - Compute look‐ahead & steering (Pure Pursuit)  
    - Compute longitudinal speed with acceleration & braking limits  
    - Publish `GaeControlCmd` & look‐ahead marker  
- **Helpers**  
  - Steering ↔ raw vehicle command mapping  
  - Throttle/brake blending based on `distance_to_stop` & `max_acceleration`  

---

## 📥 Subscribed Topics

| Topic                       | Message Type                                   | Description                          |
|-----------------------------|------------------------------------------------|--------------------------------------|
| `<trajectory_topic>`        | `nav_msgs/msg/Path`                            | Planned trajectory to follow         |
| `<pose_topic>`              | `geometry_msgs/msg/PoseWithCovarianceStamped`  | Current vehicle pose                |
| `<stop_topic>`              | `std_msgs/msg/Float64`                         | Distance until next required stop    |
| `<telemetry_topic>`         | `gae_msgs/msg/GaeTelemetry`                    | Motor velocity, vehicle status       |
| `<speed_limit_topic>`         | `std_msgs/msg/Float64`                    | Current speed limit for the vehicle       |

---

## 📤 Published Topics

| Topic                           | Message Type                 | Description                            |
|---------------------------------|------------------------------|----------------------------------------|
| `<autonomous_cmd_topic>`        | `gae_msgs/msg/GaeControlCmd` | Steering, throttle & brake commands    |
| `/look_ahead_marker`            | `visualization_msgs/Marker`  | (debug) look‐ahead target visualization |

---

## 🔧 Parameters

```yaml
controller_exe:
  ros__parameters:
    trajectory_topic: "/trajectory_planner/trajectory"
    pose_topic:          *pose_topic
    autonomous_cmd_topic: "/controller/vehicle_control"
    stop_topic:           "/mission_planner/stop_distance"
    telemetry_topic:      *telemetry_topic

    base_lookahead_distance: 8.0    # m at max speed
    sensitivity:             5.5    # steering gain
    max_speed:               70.0   # rpm-equivalent at full throttle
    min_speed:               0.0    # minimum commanded speed
    max_acceleration:        50.0   # rpm/s^2
    max_steering_angle:      90.0   # degrees
    min_lookahead_distance:  3.5    # m at low speed
    brake_distance:          30.0   # m before full stop
    wheelbase:               1.5    # vehicle wheelbase (m)
    simulation_mode:        *simulation_mode  # freeze speed from telemetry
````

---

## 🚀 Usage

```bash
ros2 run planner controller_exe \
  --ros-args --params-file /path/to/controller_config.yaml
```

---

## 💡 Notes

* In **simulation\_mode**, the node uses the last desired speed instead of real‐world telemetry.
* Visualization of the look‐ahead point can be viewed in RViz via `/look_ahead_marker`.
* In the future, other controller algorithms may be implemented.

---

## 🧪 Testing

1. **Unit tests**

   * Mock all subscribed topics; verify correct `GaeControlCmd` outputs for known inputs.
2. **Integration tests**

   * Run in Gazebo + ROS2: drive on a straight line, curves, and obstacle‐stop scenarios; inspect control outputs.
3. **Edge cases**

   * Empty trajectory → error log, no control command.
   * Rapid speed changes → enforce `max_acceleration`.
   * Very close stop distance (<3 m) → zero throttle, full brake.