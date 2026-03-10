# trajectory_planner_node

## 🎯 Purpose
The `trajectory_planner_node` generates a driv­able trajectory (`nav_msgs/Path`) from a planned lanelet sequence and an occupancy grid. It supports two modes:
- **Centerline follow**: output raw lanelet centerline points  
- **Grid‐based A\***: plan a collision‐free path on the occupancy grid, then smooth  

---

## 🔢 Algorithm
1. **Load** lanelet2 map (`.osm`) using `origin_pose`.  
2. **Subscribe** to:  
   - Current pose (`pose_topic`)  
   - Planned path (`path_topic` of type `GaePathIds`)  
   - Occupancy grid (`occupancy_grid_topic`)  
3. **On `pathCallback`**: store the latest `GaePathIds`.  
4. **On `occupancyGridCallback`**: store the latest grid.  
5. **On `poseCallback`**:  
   - If `follow_centerline == true`, call `followCenterline()`:  
     - Find the current lane in the path  
     - Publish remaining centerline points as a `Path`  
   - Else (A\* mode):  
     - Project path centerline into grid pixels  
     - Find the first collision‐free goal cell on the grid  
     - Configure A\* generator (clearance loop, curvature & centerline factors)  
     - Add occupied cells (from grid value 100/50) as collisions  
     - Find grid‐cell path from center to goal  
     - Convert grid‐cell path → world poses  
     - Smooth with moving average + spline interpolation  
     - Publish the resulting `nav_msgs/Path`  

---

## ⚙️ Inner Logic
- **Constructor**  
  - Declare & get parameters  
  - Load & project lanelet map  
  - Create subscriptions & the `path_pub`  
- **`readLaneletMap()`**: loads `.osm` via UTMC projector  
- **Callbacks**  
  - `pathCallback(msg)`: cache `GaePathIds`  
  - `occupancyGridCallback(grid)`: cache occupancy grid  
  - `poseCallback(msg)`: choose mode → call `followCenterline()` or perform grid‐A\* → publish trajectory  
- **Helpers**  
  - `followCenterline()`: streams raw centerline as `Path`  
  - `movingAverage()`: simple low-pass filter for smoothing  
  - Spline interpolation via `tk::spline`  

---

## 📥 Subscribed Topics

| Topic                       | Type                                          | Description                         |
|-----------------------------|-----------------------------------------------|-------------------------------------|
| `<pose_topic>`              | `PoseWithCovarianceStamped`                   | Current vehicle pose               |
| `<path_topic>`              | `gae_msgs/msg/GaePathIds`                     | Sequence of lanelet IDs to follow  |
| `<occupancy_grid_topic>`    | `nav_msgs/msg/OccupancyGrid`                  | Local occupancy grid for collisions |

---

## 📤 Published Topics

| Topic                       | Type                 | Description                          |
|-----------------------------|----------------------|--------------------------------------|
| `<trajectory_topic>`        | `nav_msgs/msg/Path`  | Planned trajectory for controllers   |

---

## 🔧 Parameters

```yaml
trajectory_planner_exe:
  ros__parameters:
    origin_pose:
      _x: *_x                           # map projection origin (latitude)
      _y: *_y                           # map projection origin (longitude)
    osm_path: *osm_path                 # lanelet2 OSM file path
    pose_topic: *pose_topic             # e.g. "/pose"
    follow_centerline: true             # raw centerline mode toggle
    curvature_contributing_factor: 3.0  # A* curvature weight
    centerline_contributing_factor: 7.0 # A* centerline weight
    clearance: 0                        # initial clearance in pixels
    occupancy_grid_topic: "/occupancy_grid"
    path_topic: "/mission_planner/path"
    trajectory_topic: "/trajectory_planner/trajectory"
```

---

## 🚀 Usage

```bash
ros2 run planner trajectory_planner_exe \
  --ros-args --params-file /path/to/trajectory_planner_config.yaml
```

---

## 💡 Notes

* **Centerline mode** is simpler but ignores obstacles.
* **A\*** may be slow at high `clearance`; consider dynamic adjustment or hierarchical search.
* Spline smoothing can be tuned via window size and `num_fine`.
---

## 🧪 Testing

1. **Unit tests**

   * Mock each subscription; verify published `Path` contents in both modes.
2. **Integration tests**

   * Full stack in Gazebo + RViz: confirm trajectory avoids obstacles and follows lanelets.
3. **Edge cases**

   * No path → error log, no publish.
   * No occupancy grid in A\* mode → error log, skip planning.
   * Unreachable goal → error log, no trajectory.