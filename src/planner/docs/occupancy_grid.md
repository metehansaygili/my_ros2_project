# occupancy_grid_node

## 🎯 Purpose
The `occupancy_grid_node` builds a local occupancy grid around the vehicle by combining:  
1. **Static lanelet boundaries** (from a lanelet2 map & current planned path)  
2. **Dynamic obstacles** (from clustered LiDAR detections)  
It publishes a `nav_msgs/OccupancyGrid` for downstream planners and, in debug mode, visualizes lanelet boundaries as markers.

---

## 🔢 Algorithm
1. **Load** the lanelet2 map (`.osm`) with `origin_pose`.  
2. **Subscribe** to:  
   - Vehicle pose (`/pose`)  
   - Clustered ground points (`/all_bottom_clusters`)  
   - Current planned path (`/path_ids`)  
3. **On clustersCallback**:  
   - Compute grid origin & orientation from current pose  
   - Rasterize the current path’s lanelet polygons into a **lanelet mask** (free = 0, unknown = 100)  
   - Rasterize dynamic clusters into an **obstacle mask**, inflate by vehicle footprint + margin  
   - **Combine** masks → log-odds update:  
     - Obstacle cells = 100 (occupied)  
     - Lanelet-free cells = 0 (free)  
     - Others = 50 (unknown)  
   - Publish the resulting `OccupancyGrid`.  
4. **On localization_callback**: update the current vehicle pose.  
5. **On pathCallback**: store the latest `GaePathIds` for use in lanelet masking.

---

## ⚙️ Inner Logic
- **Constructor**  
  - Declare & read ROS2 parameters  
  - Load & project lanelet2 map  
  - Create publishers & subscriptions  
- **Callback Functions**  
  - `clustersCallback(msg)` → builds & publishes the occupancy grid  
  - `localization_callback(msg)` → updates `this->pose`  
  - `pathCallback(msg)` → updates `this->path_`  
- **Utility**  
  - `readLaneletMap()` → loads the `.osm` map via UTMC projector  
  - (Private) `transform_point(...)` → maps a global point into grid pixel coordinates

---

## 📥 Subscribed Topics

| Topic             | Type                                          | Description                               |
| ----------------- | --------------------------------------------- | ----------------------------------------- |
| `<pose_topic>`    | `geometry_msgs/msg/PoseWithCovarianceStamped` | Current vehicle pose                      |
| `<cluster_topic>` | `gae_msgs/msg/GaeBottomPointsArray`           | Clustered ground‐points from LiDAR        |
| `<path_topic>`    | `gae_msgs/msg/GaePathIds`                     | Current sequence of lanelet IDs to raster |

---

## 📤 Published Topics

| Topic                          | Type                                 | Description                                   |
| ------------------------------ | ------------------------------------ | --------------------------------------------- |
| `<occupancy_grid_topic>`       | `nav_msgs/msg/OccupancyGrid`         | The computed occupancy grid                   |
| `<debug_lanelet_marker_topic>` | `visualization_msgs/msg/MarkerArray` | (debug\_mode) Lanelet boundary visualizations |

---

## 🔧 Parameters

```yaml
occupancy_grid_exe:
  ros__parameters:
    grid_resolution: 0.2               # meters per grid cell
    grid_size: 200                     # number of cells per side
    origin_pose:
      _x: *_x                         # map projection origin (latitude)
      _y: *_y                         # map projection origin (longitude)
    osm_path: *osm_path               # path to lanelet2 OSM file
    pose_topic: *pose_topic           # e.g. "/pose"
    occupancy_grid_topic: "/occupancy_grid"
    cluster_topic: "/all_bottom_clusters"
    path_topic: "/mission_planner/path"
    vehicle_width: 1.2                # vehicle width in meters
    vehicle_length: 1.2               # vehicle length in meters
    safety_margin: 0.5                # inflation margin in meters
    debug_mode: false                 # true → publish lanelet markers

````

---

## 🚀 Usage

```bash
ros2 run planner occupancy_grid_exe \
  --ros-args --params-file /path/to/occupancy_grid_config.yaml
```

---

## 💡 Notes

* **TODO:**

  * Synchronize with message filters (e.g. approximate time sync)
  * Handle lane-change logic (make right/left lanelets free)
  * Recover if the vehicle leaves all lanes (poor control)
  * Fix turn-related bugs—verify lanelet boundaries & path validity
* In **debug\_mode**, lanelet boundaries are published as LINE\_STRIP markers.

---

## 🧪 Testing

1. **Unit tests**

   * Feed synthetic `PoseWithCovarianceStamped`, `GaeBottomPointsArray`, `GaePathIds` → verify expected free/occupied cells in the grid buffer.
2. **Integration tests**

   * Run full stack in Gazebo + RViz → visualize `/occupancy_grid` and `/debug/lanelet_markers`, verify obstacle inflation and lanelet masking.
3. **Edge cases**

   * No path received → occupancy grid should default to unknown (`100`).
   * Empty clusters → only lanelet mask applied.
   * High-speed maneuvers → ensure grid origin updates correctly.