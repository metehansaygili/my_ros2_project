# mission_planner_node

## 🎯 Purpose
This node receives a goal lanelet ID and the current vehicle pose, plans a route through the lanelet2 map, re-plans on dynamic changes (e.g. obstacles, traffic rules), and publishes:
- A sequence of lanelet IDs (`GaePathIds`)  
- Utility info (e.g. stop distance) for downstream controllers  

---

## 🔢 Algorithm
1. **Load** the lanelet2 map from `osm_path` using `origin_pose`.  
2. **Listen** for a new goal on `goal_id_topic` (`std_msgs/msg/Int32`).  
3. **Determine start** lanelet by projecting `current_pose` (from `pose_topic`).  
4. **Compute** initial route using Lanelet API.
5. **Monitor** dynamic inputs (`yolo_topic`):
   - If an obstacle, closed road, or forbidden turn is detected, **re-plan**.

   Note:    (`cluster_topic` will be implemented)
6. **Publish** the results:  
   - Lanelet sequence on `path_topic` (`gae_msgs/msg/GaePathIds`)  
   - Stop/slow-down distance on `stop_topic` (`std_msgs/msg/Float64`)  

---

## ⚙️ Inner Logic
1. **Parameters**: declare & load parameters.  
2. **Map loading**: read `osm_path` → `lanelet::LaneletMap`.  
3. **Callbacks**:  
   - **`goalCallback`** (`Int32`): store goal, compute & publish route.  
   - **`poseCallback`** (`PoseWithCovarianceStamped`): update pose; publish any periodic utils.  
   - **`yoloCallback`** (`GaeCamDetectionArray`): inspect detections; if blocking, trigger re-plan.  
   - **`telemetryCallback`** (`GaeTelemetry`): use to check if the vehicle stopped.
4. **Plan Route**:  
   - Find nearest start lanelet
   - Run selected planner  
   - Convert lanelet IDs → `GaePathIds` message  

---

## 📥 Subscribed Topics

| Topic               | Message Type                                  | Description                    |
| ------------------- | --------------------------------------------- | ------------------------------ |
| `<goal_id_topic>`   | `std_msgs/msg/Int32`                          | New target lanelet ID          |
| `<pose_topic>`      | `geometry_msgs/msg/PoseWithCovarianceStamped` | Current vehicle pose           |
| `<yolo_topic>`      | `gae_sgs/msg/GaeCamDetectionArray`            | Detected obstacles (e.g. YOLO) |
| `<telemetry_topic>` | `gae_msgs/msg/GaeTelemetry`                   | Speed, brake status, etc.      |

---

## 📤 Published Topics

| Topic          | Message Type              | Description                              |
| -------------- | ------------------------- | ---------------------------------------- |
| `<path_topic>` | `gae_msgs/msg/GaePathIds` | Planned sequence of lanelet IDs          |
| `<stop_topic>` | `std_msgs/msg/Float64`    | Distance to next required stop/slow-down |

---


## 🔧 Parameters

```yaml
mission_planner_exe:
  ros__parameters:
    origin_pose: # Origin of the lanelet map to correct projection.
      _x: *_x
      _y: *_y
    osm_path: *osm_path # Path of the osm file.
    pose_topic: *pose_topic # Configurable topic names
    path_topic: "/mission_planner/path"
    goal_pose_topic: "/global_planner/goal"
    stop_topic: "/mission_planner/stop_distance"
    telemetry_topic: *telemetry_topic
    yolo_topic: "/yolo_detections"
```

---

## 🚀 Usage

```bash
ros2 run planner mission_planner_exe \
  --ros-args --params-file /path/to/mission_planner_config.yaml
```

---

## 💡 Notes

* **YOLO logic** is stubbed: implement obstacle-detection → re-plan trigger.
* **Traffic rules** (no-turn signs, closed lanes) require map-based checks.
* **Future**: Integrate the cluster callback for road closures.

---

## 🧪 Testing

1. **Unit tests**

   * Mock each subscribed topic; verify published `GaePathIds` & `stop_distance`.
2. **Integration tests**

   * Full stack in Gazebo + RViz: set goals, insert obstacles → confirm re-plan.