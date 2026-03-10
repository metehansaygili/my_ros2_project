# global_planner_node

## 🎯 Purpose
This node determines a goal lanelet ID for given scenarios or JSON files (e.g. Teknofest), and/or via RViz.  
Once computed, it publishes the goal lanelet ID to the mission planner.  
It also detects parking‐lot areas and, if applicable, computes a free parking slot from cluster data.

---

## 🔢 Algorithm
1. **Load** lanelet2 map (OSM)  
2. **Select lanelets** by attribute or JSON (todo)  
3. **Determine goal**  
   - If `use_rviz == true`, project the RViz click to a lanelet ID  
   - Otherwise, pick the next lanelet from attributes or JSON  
4. **Parking logic**  
   - In parking area (identified by `parking_lot_entry_id`):  
     - Cluster analysis → compute cluster center  
     - Match center to a lanelet → assign free slot  
5. **Publish** selected lanelet ID

---

## ⚙️ Inner Logic
1. Read `osm_path` and load lanelet2 map with `origin_pose`.  
2. Identify “station” and “park” lanelets via `station_name` / `park_name` attributes.  
3. Listen on `rviz_goal_topic` or use attribute/JSON logic to pick a lanelet ID.  
4. If in parking zone, subscribe to cluster data (`/cluster_sub`), compute center, and choose a free slot.  
5. Publish the `goal_pose_topic`.

---

## 📥 Subscribed Topics

| Topic               | Type                                          | Description                    |
| ------------------- | --------------------------------------------- | ------------------------------ |
| `<rviz_goal_topic>` | `geometry_msgs/msg/PoseStamped`               | Goal pose published by RViz    |
| `<pose_topic>`      | `geometry_msgs/msg/PoseWithCovarianceStamped` | Current vehicle pose           |
| `<cluster_topic>`   | `gae_msgs/msg/GaeBottomPointsArray`           | Detected ground‐cluster points |

---

## 📤 Published Topics

| Topic               | Type                 | Description              |
| ------------------- | -------------------- | ------------------------ |
| `<goal_pose_topic>` | `std_msgs/msg/Int32` | Selected goal lanelet ID |

---


## 🔧 Parameters

```yaml
global_planner_exe:
  ros__parameters:
    origin_pose:
      x: 0.0                                  # Lanelet map projection origin (latitude)
      y: 0.0                                  # Lanelet map projection origin (longitude)
    osm_path: ""                              # Path to the lanelet2 OSM file
    rviz_goal_topic: "/move_base_simple/goal" # Configurable topic name 
    pose_topic: *pose_topic                   # Configurable topic name
    cluster_topic: "/all_bottom_clusters"     # Configurable topic name
    station_name: "station"                   # Attributes name for the stations
    park_name: "park"                         # Attributes name for the parking slots.
    use_rviz: false                           # true → use RViz input, false → use attributes/JSON
    goal_pose_topic: "/global_planner/goal"   # Configurable topic name
    first_goal_id: 406                        # First goal id id the use_rviz is false
    parking_lot_entry_id: 115                 # Parking lot entry information.
```

---

## 🚀 Single Usage of the Module

```bash
ros2 run planner global_planner_exe --ros-args --params-file /path/to/config.yaml
```

---

## 💡 Notes

* Adjust `use_rviz` to switch between JSON/attribute logic and RViz input.
* Update `first_goal_id`/`parking_lot_entry_id` per scenario.
* Future: support JSON file input for arbitrary goal sequences.
* Ensure the lanelet map is correctly projected via `origin_pose`.

---

## 🧪 Testing

* **Unit:** Publish fake `/pose`, `/all_bottom_clusters`, and `/move_base_simple/goal` to verify each logic branch.
* **Integration:** Run full stack in Gazebo, send a goal, and confirm `/global_planner/goal` matches expectations.