#include "trajectory_planner.hpp"

namespace itusct
{
  TrajectoryPlanner::TrajectoryPlanner(const rclcpp::NodeOptions &options)
      : Node("trajectory_planner_exe", options), tf_buffer(this->get_clock()), tf_listener(tf_buffer)
  {

    RCLCPP_INFO(this->get_logger(), "Trajectory planner node is started!");
    // initialize variables
    this->declare_parameter<double>("origin_pose._x", 0.0);
    this->declare_parameter<double>("origin_pose._y", 0.0);
    this->declare_parameter<std::string>("osm_path", "./map.osm");

    this->declare_parameter<std::string>("pose_topic", "/current_pose");
    this->declare_parameter<std::string>("path_topic", "/path_ids");
    this->declare_parameter<std::string>("trajectory_topic", "/trajectory");
    this->declare_parameter<std::string>("occupancy_grid_topic", "/occupancy_grid");
    this->declare_parameter<bool>("follow_centerline", true);
    this->declare_parameter<double>("curvature_contributing_factor", 0.3);
    this->declare_parameter<int>("clearance", 5);
    this->declare_parameter<double>("centerline_contributing_factor", 0.8);
    this->declare_parameter<std::string>("switch_controller_topic", "/switch_controller");
    this->declare_parameter<std::string>("telemetry_topic", "/vehicle/telemetry");
    this->declare_parameter<double>("stop_a_star_planning_in_parking_sector_threshold", 2.5);
    this->declare_parameter<double>("stopping_distance_to_parking_spot", 0.5);
    this->declare_parameter<double>("lattice_planner_horizon", 20.0);
    this->declare_parameter<double>("max_speed", 90.0);
    this->declare_parameter<double>("min_speed", 30.0);
    this->declare_parameter<double>("lattice_max_horizon", 12.0);
    this->declare_parameter<double>("lattice_min_horizon", 6.0);

    this->get_parameter("osm_path", this->osm_path);
    this->get_parameter("origin_pose._x", this->origin_x);
    this->get_parameter("origin_pose._y", this->origin_y);
    this->get_parameter("follow_centerline", this->follow_centerline);
    this->get_parameter("curvature_contributing_factor", this->curvature_contributing_factor);
    this->get_parameter("clearance", this->clearance);
    this->get_parameter("centerline_contributing_factor", this->centerline_contributing_factor);
    this->get_parameter("stop_a_star_planning_in_parking_sector_threshold", this->stop_a_star_planning_in_parking_sector_threshold);
    this->get_parameter("stopping_distance_to_parking_spot", this->stopping_distance_to_parking_spot);
    this->get_parameter("lattice_planner_horizon", this->lattice_planner_horizon);
    this->get_parameter("max_speed", this->max_speed_);
    this->get_parameter("min_speed", this->min_speed_);
    this->get_parameter("lattice_max_horizon", this->lattice_max_horizon_);
    this->get_parameter("lattice_max_horizon", this->lattice_max_horizon_);
    this->get_parameter("lattice_min_horizon", this->lattice_min_horizon_);

    this->declare_parameter<bool>("camera_mode", false);
    this->declare_parameter<std::string>("camera_path_topic", "/drivable_area/path_base_link");
    this->get_parameter("camera_mode", this->camera_mode);
    this->get_parameter("camera_path_topic", this->camera_path_topic);

    std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();
    std::string PATH_TOPIC = this->get_parameter("path_topic").as_string();
    std::string TRAJECTORY_TOPIC = this->get_parameter("trajectory_topic").as_string();
    std::string OCCUPANCY_GRID_TOPIC = this->get_parameter("occupancy_grid_topic").as_string();
    std::string SWITCH_CONTROLLER_TOPIC = this->get_parameter("switch_controller_topic").as_string();
    std::string telemetry_topic_param = this->get_parameter("telemetry_topic").as_string();

    // read lanelet map
    this->readLaneletMap();

    // Print all parameters for debugging
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "osm_path: %s", this->osm_path.c_str());
    RCLCPP_INFO(this->get_logger(), "origin_pose._x: %f", this->origin_x);
    RCLCPP_INFO(this->get_logger(), "origin_pose._y: %f", this->origin_y);
    RCLCPP_INFO(this->get_logger(), "follow_centerline: %s", this->follow_centerline ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "curvature_contributing_factor: %f", this->curvature_contributing_factor);
    RCLCPP_INFO(this->get_logger(), "clearance: %d", this->clearance);
    RCLCPP_INFO(this->get_logger(), "centerline_contributing_factor: %f", this->centerline_contributing_factor);
    RCLCPP_INFO(this->get_logger(), "stop_a_star_planning_in_parking_sector_threshold: %f", this->stop_a_star_planning_in_parking_sector_threshold);
    RCLCPP_INFO(this->get_logger(), "stopping_distance_to_parking_spot: %f", this->stopping_distance_to_parking_spot);
    RCLCPP_INFO(this->get_logger(), "lattice_planner_horizon: %f", this->lattice_planner_horizon);
    RCLCPP_INFO(this->get_logger(), "max_speed: %f", this->max_speed_);
    RCLCPP_INFO(this->get_logger(), "min_speed: %f", this->min_speed_);
    RCLCPP_INFO(this->get_logger(), "lattice_max_horizon: %f", this->lattice_max_horizon_);
    RCLCPP_INFO(this->get_logger(), "lattice_min_horizon: %f", this->lattice_min_horizon_);

    // initialize subscribers and publishers
    debug_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/lattice_paths", 10);

    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(POSE_TOPIC, 10, std::bind(&TrajectoryPlanner::poseCallback, this, std::placeholders::_1));
    this->path_sub = this->create_subscription<gae_msgs::msg::GaePathIds>(PATH_TOPIC, 10, std::bind(&TrajectoryPlanner::pathCallback, this, std::placeholders::_1));
    this->grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(OCCUPANCY_GRID_TOPIC, 10, std::bind(&TrajectoryPlanner::occupancyGridCallback, this, std::placeholders::_1));
    this->telemetry_sub_ = this->create_subscription<gae_msgs::msg::GaeTelemetry>(telemetry_topic_param, 10, std::bind(&TrajectoryPlanner::telemetryCallback, this, std::placeholders::_1));
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>(TRAJECTORY_TOPIC, 10);
    this->state_topic_sub = this->create_subscription<std_msgs::msg::Int32>("/vehicle_state", 10, std::bind(&TrajectoryPlanner::switchControllerCallback, this, std::placeholders::_1));
    this->parking_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/global_planner/final_parking_pose", 10, std::bind(&TrajectoryPlanner::parkingGoalCallback, this, std::placeholders::_1));
    
    if(this->camera_mode){
      RCLCPP_INFO(this->get_logger(), "Camera Mode Active. Subscribing to: %s", this->camera_path_topic.c_str());
      this->camera_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
          this->camera_path_topic, 10, std::bind(&TrajectoryPlanner::cameraPathCallback, this, std::placeholders::_1));
    }

    this->stop_pub = this->create_publisher<std_msgs::msg::Bool>("/stop", 10);
  }

  void TrajectoryPlanner::switchControllerCallback(const std_msgs::msg::Int32 &msg)
  {
    switch (static_cast<planner::State>(msg.data))
    {
    case planner::State::CENTERLINETRACK:
      this->current_state_ = planner::State::CENTERLINETRACK;
      break;
    case planner::State::LATTICE:
      this->current_state_ = planner::State::LATTICE;
      break;
    case planner::State::ASTAR:
      this->current_state_ = planner::State::ASTAR;
      break;
    case planner::State::PARKING_ASTAR:
      this->current_state_ = planner::State::PARKING_ASTAR;
      break;
    default:
      this->current_state_ = planner::State::CENTERLINETRACK;
      break;
    }
  }

  void TrajectoryPlanner::parkingGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // ORIGINAL CALLBACK (commented for testing):
    // this->current_parking_goal_ = *msg;
    // this->has_parking_goal_ = true;
    // RCLCPP_INFO(this->get_logger(), "Trajectory Planner Received Parking Goal: [%f, %f]", msg->pose.position.x, msg->pose.position.y);
    
    // TEST VALUES - Hardcoded parking spot
    geometry_msgs::msg::PoseStamped hardcoded_goal;
    hardcoded_goal.header.frame_id = "map";
    hardcoded_goal.header.stamp = this->get_clock()->now();
    hardcoded_goal.pose.position.x = 33960.17977016726;
    hardcoded_goal.pose.position.y = 22.63861518841633;
    hardcoded_goal.pose.position.z = 0.0;
    
    // Set orientation from theta = -0.15 radians
    double theta = -0.15;
    hardcoded_goal.pose.orientation.x = 0.0;
    hardcoded_goal.pose.orientation.y = 0.0;
    hardcoded_goal.pose.orientation.z = std::sin(theta / 2.0);
    hardcoded_goal.pose.orientation.w = std::cos(theta / 2.0);
    
    this->current_parking_goal_ = hardcoded_goal;
    this->has_parking_goal_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Trajectory Planner Using HARDCODED Parking Goal: [%f, %f, theta=%f]", 
                hardcoded_goal.pose.position.x, hardcoded_goal.pose.position.y, theta);
  }

  void TrajectoryPlanner::telemetryCallback(const gae_msgs::msg::GaeTelemetry::SharedPtr msg)
  {
    this->current_speed_ = static_cast<double>(msg->motor_velocity);
  }

  void TrajectoryPlanner::cameraPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
      if(!this->camera_mode) return;
      
      this->latest_camera_path_ = *msg;

      // In Camera Mode, we force Lattice Planning
      // We don't rely on 'poseCallback' because localization might be unavailable.
      // Instead, we trigger the planning cycle here, using an Identity Pose for the ego vehicle (since we are in base_link)
      
      this->current_state_ = planner::State::LATTICE;

      geometry_msgs::msg::PoseWithCovarianceStamped dummy_pose;
      dummy_pose.header.stamp = this->get_clock()->now();
      dummy_pose.header.frame_id = "base_link";
      dummy_pose.pose.pose.position.x = 0.0;
      dummy_pose.pose.pose.position.y = 0.0;
      dummy_pose.pose.pose.position.z = 0.0;
      dummy_pose.pose.pose.orientation.w = 1.0;
      dummy_pose.pose.pose.orientation.x = 0.0;
      dummy_pose.pose.pose.orientation.y = 0.0;
      dummy_pose.pose.pose.orientation.z = 0.0;

      // Trigger Lattice State logic
      // This will use 'latest_camera_path_' as centerline and 'dummy_pose' as ego state
      this->runLatticeState(dummy_pose);
  }

  void TrajectoryPlanner::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    if (!this->path || this->path->lanes.size() == 0)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No path!");
      return;
    }
    // STATE MACHINE SWITCH
    switch (this->current_state_)
    {
    case planner::State::CENTERLINETRACK:
      runCenterlineState(msg);
      break;
    case planner::State::LATTICE:
      runLatticeState(msg);
      break;
    case planner::State::ASTAR:
      runAStarState(msg);
      break;
    case planner::State::PARKING_ASTAR:
      runParkingAStarState(msg);
      break;
    default:
      runCenterlineState(msg);
      break;
    }
  }

  // =======================================================================================
  // STATE 1: CENTERLINE FOLLOWING
  // =======================================================================================
  void TrajectoryPlanner::runCenterlineState(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    this->followCenterline(msg);
  }

  // =======================================================================================
  // STATE 2: LATTICE PLANNING
  // =======================================================================================
  void TrajectoryPlanner::runLatticeState(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    if (this->occupancy_grid.data.size() == 0)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No occupancy grid data!");
      return;
    }

    // 1. Get Centerline Points
    std::vector<geometry_msgs::msg::PointStamped> centerline_point_ids;

    if (this->camera_mode)
    {
        if (this->latest_camera_path_.poses.empty())
        {
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Camera mode active but no path received yet.");
             return;
        }

        for (const auto& pose : this->latest_camera_path_.poses)
        {
            geometry_msgs::msg::PointStamped p;
            p.header = this->latest_camera_path_.header; // Should be base_link
            p.point = pose.pose.position;
            centerline_point_ids.push_back(p);
        }
    }
    else
    {
        lanelet::BasicPoint2d current_point(msg.pose.pose.position.x, msg.pose.pose.position.y);
        // Note: Finding nearest lanelet logic
        if (this->lanelet_map->laneletLayer.size() == 0)
        return;
        auto nearest = this->lanelet_map->laneletLayer.nearest(current_point, 1);
        if (nearest.empty())
        return;

        for (size_t i = 0; i < this->path->lanes.size(); i++)
        {
        const lanelet::ConstLanelet current_lanelet = this->lanelet_map->laneletLayer.get(this->path->lanes[i]);
        for (auto point : current_lanelet.centerline())
        {
            geometry_msgs::msg::PointStamped point_;
            point_.header.frame_id = msg.header.frame_id;
            point_.point.x = point.x();
            point_.point.y = point.y();
            point_.point.z = point.z();
            centerline_point_ids.push_back(point_);
        }
        }
    }

    if (centerline_point_ids.size() == 0)
      return;

    // 2. Find Current Index
    double distance = std::numeric_limits<double>::max();
    size_t current_idx = 0;
    
    // In Camera Mode, vehicle is at (0,0) in base_link
    double search_x = this->camera_mode ? 0.0 : msg.pose.pose.position.x;
    double search_y = this->camera_mode ? 0.0 : msg.pose.pose.position.y;

    for (size_t i = 0; i < centerline_point_ids.size(); i++)
    {
      double d = std::hypot(centerline_point_ids[i].point.x - search_x,
                            centerline_point_ids[i].point.y - search_y);
      if (d < distance)
      {
        distance = d;
        current_idx = i;
      }
    }

    // 3. Lattice Generator Setup
    static Lattice::Generator lattice_planner;
    lattice_planner.setGridInfo(
        this->occupancy_grid.info.width, this->occupancy_grid.info.height,
        this->occupancy_grid.info.resolution,
        this->occupancy_grid.info.origin.position.x, this->occupancy_grid.info.origin.position.y,
        this->occupancy_grid.data);

    lattice_planner.set_speed_limits(this->min_speed_, this->max_speed_);
    lattice_planner.setLookaheadDistances(this->lattice_min_horizon_, this->lattice_max_horizon_);

    // 4. Prepare Local Centerline
    std::vector<Lattice::Point> local_centerline;
    double car_yaw = this->camera_mode ? 0.0 : tf2::getYaw(msg.pose.pose.orientation);
    double car_x = this->camera_mode ? 0.0 : msg.pose.pose.position.x;
    double car_y = this->camera_mode ? 0.0 : msg.pose.pose.position.y;

    size_t start_search_idx = (current_idx > 10) ? (current_idx - 10) : 0;
    size_t end_search_idx = std::min(current_idx + 60, centerline_point_ids.size());

    for (size_t i = start_search_idx; i < end_search_idx; i++)
    {
      double gx = centerline_point_ids[i].point.x;
      double gy = centerline_point_ids[i].point.y;
      double dx = gx - car_x;
      double dy = gy - car_y;
      double lx = dx * std::cos(-car_yaw) - dy * std::sin(-car_yaw);
      double ly = dx * std::sin(-car_yaw) + dy * std::cos(-car_yaw);
      double dist = std::hypot(lx, ly);

      if (lx > 0.0 && dist <= this->lattice_planner_horizon)
      {
        local_centerline.push_back({lx, ly});
      }
    }
    lattice_planner.setCenterline(local_centerline);

    // 5. Compute
    std::vector<Lattice::Point> best_path_local = lattice_planner.computeTrajectory(car_x, car_y, car_yaw, this->current_speed_, this->get_logger());

    std::string output_frame = this->camera_mode ? "base_link" : "map";

    // 6. Visualization
    if (this->debug_paths_pub_->get_subscription_count() > 0)
    {
      visualization_msgs::msg::MarkerArray markers;
      const auto &all_paths = lattice_planner.getAllTrajectories();
      int id_counter = 0;
      for (const auto &path : all_paths)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = output_frame;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "lattice_candidates";
        marker.id = id_counter++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = 0;
        marker.scale.x = 0.05;
        if (path.blocked)
        {
          marker.color.r = 1.0;
          marker.color.a = 0.3;
        }
        else
        {
          marker.color.b = 1.0;
          marker.color.a = 0.5;
        }
        for (const auto &pt : path.points)
        {
          geometry_msgs::msg::Point p;
          p.x = pt.x * std::cos(car_yaw) - pt.y * std::sin(car_yaw) + car_x;
          p.y = pt.x * std::sin(car_yaw) + pt.y * std::cos(car_yaw) + car_y;
          p.z = 0.0;
          marker.points.push_back(p);
        }
        markers.markers.push_back(marker);
      }
      if (!best_path_local.empty())
      {
        visualization_msgs::msg::Marker best_m;
        best_m.header.frame_id = output_frame;
        best_m.header.stamp = this->get_clock()->now();
        best_m.ns = "lattice_best";
        best_m.id = 999;
        best_m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        best_m.action = 0;
        best_m.scale.x = 0.2;
        best_m.color.g = 1.0;
        best_m.color.a = 1.0;
        for (const auto &pt : best_path_local)
        {
          geometry_msgs::msg::Point p;
          p.x = pt.x * std::cos(car_yaw) - pt.y * std::sin(car_yaw) + car_x;
          p.y = pt.x * std::sin(car_yaw) + pt.y * std::cos(car_yaw) + car_y;
          p.z = 0.1;
          best_m.points.push_back(p);
        }
        markers.markers.push_back(best_m);
      }
      this->debug_paths_pub_->publish(markers);
    }

    if (best_path_local.empty())
      return;

    // 7. Convert to Global
    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = output_frame;
    trajectory.header.stamp = this->get_clock()->now();

    for (const auto &pt : best_path_local)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = trajectory.header;
      double gx = pt.x * std::cos(car_yaw) - pt.y * std::sin(car_yaw) + car_x;
      double gy = pt.x * std::sin(car_yaw) + pt.y * std::cos(car_yaw) + car_y;
      pose.pose.position.x = gx;
      pose.pose.position.y = gy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      trajectory.poses.push_back(pose);
    }

    // 8. Smooth and Publish (Shared Logic)
    std::vector<double> x_coords, y_coords;
    for (auto pose : trajectory.poses)
    {
      x_coords.push_back(pose.pose.position.x);
      y_coords.push_back(pose.pose.position.y);
    }

    int window_size = 3;
    std::vector<double> x_smoothed = this->movingAverage(x_coords, window_size);
    std::vector<double> y_smoothed = this->movingAverage(y_coords, window_size);

    if (x_smoothed.size() < 3 || y_smoothed.size() < 3)
    {
      // Compute orientations for raw trajectory before publishing
      for (size_t i = 0; i < trajectory.poses.size(); ++i)
      {
        double yaw = 0.0;
        if (i + 1 < trajectory.poses.size())
        {
          double dx = trajectory.poses[i + 1].pose.position.x - trajectory.poses[i].pose.position.x;
          double dy = trajectory.poses[i + 1].pose.position.y - trajectory.poses[i].pose.position.y;
          yaw = std::atan2(dy, dx);
        }
        else if (i > 0)
        {
          double dx = trajectory.poses[i].pose.position.x - trajectory.poses[i - 1].pose.position.x;
          double dy = trajectory.poses[i].pose.position.y - trajectory.poses[i - 1].pose.position.y;
          yaw = std::atan2(dy, dx);
        }
        trajectory.poses[i].pose.orientation.x = 0.0;
        trajectory.poses[i].pose.orientation.y = 0.0;
        trajectory.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
        trajectory.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
      }
      this->path_pub->publish(trajectory);
    }
    else
    {
      std::vector<double> distances;
      distances.push_back(0.0);
      for (size_t i = 1; i < x_smoothed.size(); ++i)
      {
        double d = std::hypot(x_smoothed[i] - x_smoothed[i - 1], y_smoothed[i] - y_smoothed[i - 1]);
        distances.push_back(distances.back() + d);
      }

      // Check if path has sufficient length
      if (distances.back() < 1e-6)
      {
        RCLCPP_ERROR(this->get_logger(), "Path has zero or near-zero length, cannot interpolate!");
        this->path_pub->publish(trajectory);
        return;
      }

      std::vector<double> t;
      std::vector<double> x_filtered;
      std::vector<double> y_filtered;
      for (size_t i = 0; i < distances.size(); ++i)
      {
        double t_val = distances[i] / distances.back();
        // Only add point if it's the first or if it increases the parameter
        if (t.empty() || t_val > t.back() + 1e-9)
        {
          t.push_back(t_val);
          x_filtered.push_back(x_smoothed[i]);
          y_filtered.push_back(y_smoothed[i]);
        }
      }

      // Need at least 2 points for spline
      if (t.size() < 2)
      {
        RCLCPP_ERROR(this->get_logger(), "Not enough unique points for spline interpolation!");
        this->path_pub->publish(trajectory);
        return;
      }

      tk::spline spline_x;
      tk::spline spline_y;
      spline_x.set_points(t, x_filtered);
      spline_y.set_points(t, y_filtered);

      std::vector<geometry_msgs::msg::PoseStamped> interpolated_path;
      for (int i = 0; i < 50; ++i)
      {
        double ti = static_cast<double>(i) / 49.0;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = this->occupancy_grid.header.frame_id;
        pose.pose.position.x = spline_x(ti);
        pose.pose.position.y = spline_y(ti);
        pose.pose.position.z = 0.0;
        
        // Compute orientation from spline derivatives for proper MPC tracking
        double dx = spline_x.deriv(1, ti);  // First derivative dx/dt
        double dy = spline_y.deriv(1, ti);  // First derivative dy/dt
        double yaw = std::atan2(dy, dx);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        
        interpolated_path.push_back(pose);
      }
      nav_msgs::msg::Path interpolated_path_msg;
      interpolated_path_msg.header = trajectory.header;
      interpolated_path_msg.poses = interpolated_path;
      this->path_pub->publish(interpolated_path_msg);
    }
  }

  // =======================================================================================
  // STATE 3: A* PLANNING
  // =======================================================================================
  void TrajectoryPlanner::runAStarState(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    if (this->occupancy_grid.data.size() == 0)
      return;

    double distance_to_center_pose = std::hypot(
        this->current_parking_goal_.pose.position.x - msg.pose.pose.position.x,
        this->current_parking_goal_.pose.position.y - msg.pose.pose.position.y);

    if (this->current_state_ == planner::State::PARKING_ASTAR && distance_to_center_pose < 2.0)
    {
      RCLCPP_INFO(this->get_logger(), "A* to Parking Goal: Within 2 meters of parking goal, no need to plan.");
      return;
    }

    // 1. Centerline & Goal Finding (Required for A* to know where to go)
    std::vector<geometry_msgs::msg::PointStamped> centerline_point_ids;
    lanelet::BasicPoint2d current_point(msg.pose.pose.position.x, msg.pose.pose.position.y);
    if (this->lanelet_map->laneletLayer.size() == 0)
      return;
    auto nearest = this->lanelet_map->laneletLayer.nearest(current_point, 1);
    if (nearest.empty())
      return;

    for (size_t i = 0; i < this->path->lanes.size(); i++)
    {
      const lanelet::ConstLanelet current_lanelet = this->lanelet_map->laneletLayer.get(this->path->lanes[i]);
      for (auto point : current_lanelet.centerline())
      {
        geometry_msgs::msg::PointStamped point_;
        point_.header.frame_id = msg.header.frame_id;
        point_.point.x = point.x();
        point_.point.y = point.y();
        point_.point.z = point.z();
        centerline_point_ids.push_back(point_);
      }
    }
    if (centerline_point_ids.size() == 0)
      return;

    double distance = std::numeric_limits<double>::max();
    size_t current_idx = 0;
    for (size_t i = 0; i < centerline_point_ids.size(); i++)
    {
      double d = std::hypot(centerline_point_ids[i].point.x - msg.pose.pose.position.x,
                            centerline_point_ids[i].point.y - msg.pose.pose.position.y);
      if (d < distance)
      {
        distance = d;
        current_idx = i;
      }
    }

    double yaw = tf2::getYaw(msg.pose.pose.orientation);
    int goal_x = 0;
    int goal_y = 0;
    int goal_idx = 0;
    bool found = false;
    const size_t LOOKAHEAD_POINTS = 20;
    const size_t MAX_SEARCH_RANGE = 20;
    size_t search_end = std::min(current_idx + MAX_SEARCH_RANGE, centerline_point_ids.size());

    for (size_t i = current_idx; i < search_end; i++)
    {
      double dx = centerline_point_ids[i].point.x - this->occupancy_grid.info.origin.position.x;
      double dy = centerline_point_ids[i].point.y - this->occupancy_grid.info.origin.position.y;
      double rot_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
      double rot_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);
      unsigned int idx = static_cast<unsigned int>(rot_x / this->occupancy_grid.info.resolution);
      unsigned int idy = static_cast<unsigned int>(rot_y / this->occupancy_grid.info.resolution);
      if (idx < this->occupancy_grid.info.width && idy < this->occupancy_grid.info.height)
      {
        unsigned int grid_idx = idy * this->occupancy_grid.info.width + idx;
        if (this->occupancy_grid.data[grid_idx] == 0)
        {
          goal_x = idx;
          goal_y = idy;
          goal_idx = i;
          found = true;
          if (i >= current_idx + LOOKAHEAD_POINTS)
            break;
        }
      }
    }

    if (!found && search_end < centerline_point_ids.size())
    {
      size_t extended_end = std::min(search_end + 10, centerline_point_ids.size());
      for (size_t i = search_end; i < extended_end; i++)
      {
        double dx = centerline_point_ids[i].point.x - this->occupancy_grid.info.origin.position.x;
        double dy = centerline_point_ids[i].point.y - this->occupancy_grid.info.origin.position.y;
        double rot_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
        double rot_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);
        unsigned int idx = static_cast<unsigned int>(rot_x / this->occupancy_grid.info.resolution);
        unsigned int idy = static_cast<unsigned int>(rot_y / this->occupancy_grid.info.resolution);
        if (idx < this->occupancy_grid.info.width && idy < this->occupancy_grid.info.height)
        {
          unsigned int grid_idx = idy * this->occupancy_grid.info.width + idx;
          if (this->occupancy_grid.data[grid_idx] == 0)
          {
            goal_x = idx;
            goal_y = idy;
            goal_idx = i;
            found = true;
            break;
          }
        }
      }
    }

    if (!found)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "A*: No goal point found in the occupancy grid!");
      return;
    }

    // 2. A* Execution
    // Start heading in local grid (vehicle aligned) is 0.0
    double startHeading = 0.0;
    double goalHeading = 0.0;

    // Calculate Goal Heading relative to vehicle yaw
    if (centerline_point_ids.size() > 0 && goal_idx > 0)
    {
      double gdy = centerline_point_ids[goal_idx].point.y - centerline_point_ids[goal_idx - 1].point.y;
      double gdx = centerline_point_ids[goal_idx].point.x - centerline_point_ids[goal_idx - 1].point.x;
      double global_heading = std::atan2(gdy, gdx);
      goalHeading = global_heading - yaw; // Relative to car
    }

    static AStar::Generator generator;
    generator.setWorldSize({static_cast<int>(this->occupancy_grid.info.width), static_cast<int>(this->occupancy_grid.info.height)});
    generator.clearCollisions();
    generator.setCurvatureContributingFactor(this->curvature_contributing_factor);

    // Centerline Heuristic (Grid Coords)
    std::vector<Vec2i> centerline;
    for (size_t i = 0; i < centerline_point_ids.size(); i++)
    {
      double dx = centerline_point_ids[i].point.x - this->occupancy_grid.info.origin.position.x;
      double dy = centerline_point_ids[i].point.y - this->occupancy_grid.info.origin.position.y;
      double rot_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
      double rot_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);
      unsigned idx = rot_x / this->occupancy_grid.info.resolution;
      unsigned idy = rot_y / this->occupancy_grid.info.resolution;
      if (idx < this->occupancy_grid.info.width && idy < this->occupancy_grid.info.height)
      {
        centerline.push_back({static_cast<int>(idx), static_cast<int>(idy)});
      }
    }
    generator.setCenterlineContributingFactor(this->centerline_contributing_factor);
    generator.setCenterline(centerline);

    unsigned int start_x = this->occupancy_grid.info.width / 2;
    unsigned int start_y = this->occupancy_grid.info.height / 2;

    for (unsigned int y = 0; y < this->occupancy_grid.info.width; y++)
    {
      for (unsigned int x = 0; x < this->occupancy_grid.info.height; x++)
      {
        unsigned int idx = y * this->occupancy_grid.info.width + x;
        if (this->occupancy_grid.data[idx] > 30)
        {
          if (idx != start_y * this->occupancy_grid.info.width + start_x)
            generator.addCollision({static_cast<int>(x), static_cast<int>(y)});
        }
      }
    }

    Vec2i startCoord = {static_cast<int>(start_x), static_cast<int>(start_y)};
    generator.removeCollision({goal_x, goal_y});
    Vec2i goalCoord = {goal_x, goal_y};

    static int cycle_count = 0;
    cycle_count++;
    if (cycle_count % 3 != 0)
      return;

    generator.setMinTurningRadius(6.0);
    static std::vector<std::shared_ptr<AStar::Node>> path;
    int originalClearance = this->clearance;
    bool pathFound = false;

    // Iterative Clearance
    for (int currentClearance = originalClearance; currentClearance >= 0; currentClearance--)
    {
      generator.setClearance(currentClearance);
      path = generator.findPath(startCoord, startHeading, goalCoord, goalHeading);
      if (!path.empty() && path.back()->coordinates == goalCoord)
      {
        pathFound = true;
        break;
      }
    }

    // Iterative Radius
    if (!pathFound)
    {
      generator.setClearance(0);
      for (double r = 4.5; r >= 0.5; r -= 1.0)
      {
        generator.setMinTurningRadius(r);
        path = generator.findPath(startCoord, startHeading, goalCoord, goalHeading);
        if (!path.empty() && path.back()->coordinates == goalCoord)
        {
          pathFound = true;
          break;
        }
      }
    }

    if (!pathFound)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "A* Failed to find path.");
      return;
    }

    // 3. Output Generation & Smoothing
    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = this->get_clock()->now();

    for (auto node : path)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = trajectory.header;

      // Grid -> Local
      double rot_x = node->coordinates.x * this->occupancy_grid.info.resolution;
      double rot_y = node->coordinates.y * this->occupancy_grid.info.resolution;
      // Local -> Global (undo rotation)
      double dx = std::cos(yaw) * rot_x - std::sin(yaw) * rot_y;
      double dy = std::sin(yaw) * rot_x + std::cos(yaw) * rot_y;
      // Translate
      pose.pose.position.x = dx + this->occupancy_grid.info.origin.position.x;
      pose.pose.position.y = dy + this->occupancy_grid.info.origin.position.y;
      pose.pose.position.z = 0.0;

      // Calculate orientation
      if (trajectory.poses.size() > 0)
      {
        auto &prev_pose = trajectory.poses.back();
        double pdx = pose.pose.position.x - prev_pose.pose.position.x;
        double pdy = pose.pose.position.y - prev_pose.pose.position.y;
        double pyaw = std::atan2(pdy, pdx);
        pose.pose.orientation.z = std::sin(pyaw / 2.0);
        pose.pose.orientation.w = std::cos(pyaw / 2.0);
      }
      else
      {
        pose.pose.orientation.w = 1.0;
      }
      trajectory.poses.push_back(pose);
    }

    // Smooth
    std::vector<double> x_coords, y_coords;
    for (auto pose : trajectory.poses)
    {
      x_coords.push_back(pose.pose.position.x);
      y_coords.push_back(pose.pose.position.y);
    }
    int window_size = 3;
    std::vector<double> x_smoothed = this->movingAverage(x_coords, window_size);
    std::vector<double> y_smoothed = this->movingAverage(y_coords, window_size);

    if (x_smoothed.size() < 3)
    {
      this->path_pub->publish(trajectory);
    }
    else
    {
      std::vector<double> distances;
      distances.push_back(0.0);
      for (size_t i = 1; i < x_smoothed.size(); ++i)
      {
        double d = std::hypot(x_smoothed[i] - x_smoothed[i - 1], y_smoothed[i] - y_smoothed[i - 1]);
        distances.push_back(distances.back() + d);
      }

      // Check if path has sufficient length
      if (distances.back() < 1e-6)
      {
        RCLCPP_ERROR(this->get_logger(), "Path has zero or near-zero length, cannot interpolate!");
        this->path_pub->publish(trajectory);
        return;
      }

      std::vector<double> t;
      std::vector<double> x_filtered;
      std::vector<double> y_filtered;
      for (size_t i = 0; i < distances.size(); ++i)
      {
        double t_val = distances[i] / distances.back();
        // Only add point if it's the first or if it increases the parameter
        if (t.empty() || t_val > t.back() + 1e-9)
        {
          t.push_back(t_val);
          x_filtered.push_back(x_smoothed[i]);
          y_filtered.push_back(y_smoothed[i]);
        }
      }

      // Need at least 2 points for spline
      if (t.size() < 2)
      {
        RCLCPP_ERROR(this->get_logger(), "Not enough unique points for spline interpolation!");
        this->path_pub->publish(trajectory);
        return;
      }

      tk::spline spline_x;
      tk::spline spline_y;
      spline_x.set_points(t, x_filtered);
      spline_y.set_points(t, y_filtered);

      std::vector<geometry_msgs::msg::PoseStamped> interpolated_path;
      for (int i = 0; i < 50; ++i)
      {
        double ti = static_cast<double>(i) / 49.0;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = trajectory.header;
        pose.pose.position.x = spline_x(ti);
        pose.pose.position.y = spline_y(ti);
        pose.pose.position.z = 0.0;
        
        // Compute orientation from spline derivatives for proper MPC tracking
        double dx = spline_x.deriv(1, ti);  // First derivative dx/dt
        double dy = spline_y.deriv(1, ti);  // First derivative dy/dt
        double yaw = std::atan2(dy, dx);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        
        interpolated_path.push_back(pose);
      }
      nav_msgs::msg::Path interpolated_msg;
      interpolated_msg.header = trajectory.header;
      interpolated_msg.poses = interpolated_path;
      this->path_pub->publish(interpolated_msg);
    }
  }

  // =======================================================================================
  // STATE 4: PARKING WITH DUBINS PATH PLANNING
  // =======================================================================================
  void TrajectoryPlanner::runParkingAStarState(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    // Calculate distance to parking goal
    double distance_to_center_pose = std::hypot(
        this->current_parking_goal_.pose.position.x - msg.pose.pose.position.x,
        this->current_parking_goal_.pose.position.y - msg.pose.pose.position.y);

    // Check if we've reached the goal
    if (distance_to_center_pose < this->stopping_distance_to_parking_spot)
    {
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = true;
      this->stop_pub->publish(stop_msg);
      return;
    }

    // Skip planning when very close to avoid jitter
    if (distance_to_center_pose < this->stopping_distance_to_parking_spot)
    {
      return;
    }

    // if (distance_to_center_pose < 5.0) //dubins kapama
    // {
    //   RCLCPP_INFO(this->get_logger(), "Car to Parking Goal: Within 5 meters of parking goal, no need to plan.");
    //   return;
    // }

    // Check prerequisites
    if (this->occupancy_grid.data.size() == 0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for occupancy grid...");
      return;
    }

    if (!this->has_parking_goal_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "In PARKING STATE but NO PARKING GOAL received!");
      return;
    }

    // ============================
    // PROFESSIONAL PARKING - STABLE PATH WITH MINIMAL REPLANNING
    // ============================

    // Path stability control - LOCK path once computed until goal changes significantly
    static int replan_counter = 0;
    static double last_distance = 999.0;
    static nav_msgs::msg::Path last_valid_path;
    static bool has_valid_path = false;
    static geometry_msgs::msg::PoseStamped locked_goal;
    static bool goal_locked = false;

    replan_counter++;

    // Check if goal has changed significantly (new detection)
    bool goal_changed = false;
    if (goal_locked)
    {
      double goal_position_change = std::hypot(
          this->current_parking_goal_.pose.position.x - locked_goal.pose.position.x,
          this->current_parking_goal_.pose.position.y - locked_goal.pose.position.y);

      if (goal_position_change > 0.5)
      {
        goal_changed = true;
        RCLCPP_INFO(this->get_logger(), "Goal changed significantly - replanning");
      }
    }

    // VERY CONSERVATIVE replanning - only when absolutely necessary
    int replan_threshold = 999; // Almost never by default

    if (distance_to_center_pose > 10.0)
    {
      replan_threshold = 5; // Far: replan every 5 cycles
    }
    else if (distance_to_center_pose > 5.0)
    {
      replan_threshold = 10; // Medium: replan every 10 cycles
    }
    else
    {
      replan_threshold = 999; // Very close: NEVER replan, just follow path
    }

    // Use cached path if available and goal hasn't changed
    bool making_progress = (distance_to_center_pose < last_distance - 0.05);
    if (has_valid_path && !goal_changed && (replan_counter % replan_threshold != 0))
    {
      // Keep using same path for stability
      this->path_pub->publish(last_valid_path);
      last_distance = distance_to_center_pose;
      return;
    }

    // Only replan if needed
    if (goal_changed || !has_valid_path || (replan_counter % replan_threshold == 0))
    {
      RCLCPP_INFO(this->get_logger(), "REPLANNING at dist=%.2fm", distance_to_center_pose);
      locked_goal = this->current_parking_goal_;
      goal_locked = true;
    }

    last_distance = distance_to_center_pose;

    // Setup Dubins planner with SAFE, SMOOTH parameters for real vehicle
    static DubinsParking::ParkingPlanner dubins_planner;

    // REAL VEHICLE SAFE PARAMETERS
    double safe_radius = 5.0; // Large radius = gentle, safe curves

    dubins_planner.setMinTurningRadius(safe_radius);
    dubins_planner.setStepSize(0.15);              // Smoother waypoints
    dubins_planner.setDirectThresholds(0.8, 0.08); // Almost never use simplified
    dubins_planner.setPathEfficiencyPenalty(3.0);  // Allow longer paths for smoothness
    dubins_planner.setSmoothPathBias(0.9);         // VERY strong smooth preference
    dubins_planner.setLateTurnPenaltyFactor(1.0);  // MAXIMUM penalty on late turns
    dubins_planner.setEarlyCurveBias(1.6);         // Start curves VERY early

    // Get current vehicle pose
    double current_x = msg.pose.pose.position.x;
    double current_y = msg.pose.pose.position.y;
    double current_theta = tf2::getYaw(msg.pose.pose.orientation);

    // Get parking goal pose - USE LOCKED GOAL for stability
    double goal_x = locked_goal.pose.position.x;
    double goal_y = locked_goal.pose.position.y;
    double goal_theta = -0.15;

    // NO STAGING - Direct Dubins path with early orientation alignment
    // The Dubins algorithm with strong early-turn bias will handle orientation naturally

    DubinsParking::Pose start_pose(current_x, current_y, current_theta);
    DubinsParking::Pose goal_pose(goal_x, goal_y, goal_theta);

    // Calculate orientation difference
    double orientation_diff = goal_theta - current_theta;
    while (orientation_diff > M_PI)
      orientation_diff -= 2.0 * M_PI;
    while (orientation_diff < -M_PI)
      orientation_diff += 2.0 * M_PI;

    // DEBUG: Log planning inputs
    RCLCPP_INFO(this->get_logger(),
                "PLANNING: dist=%.2fm, orient_diff=%.1f°, radius=%.1fm",
                distance_to_center_pose, orientation_diff * 180.0 / M_PI, safe_radius);

    // Plan Dubins path
    DubinsParking::DubinsPath dubins_path = dubins_planner.planPath(start_pose, goal_pose);

    if (!dubins_path.valid)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dubins path planning failed!");

      // Use last valid path if available
      if (has_valid_path)
      {
        RCLCPP_WARN(this->get_logger(), "Using last valid path as fallback");
        this->path_pub->publish(last_valid_path);
      }
      return;
    }

    // Log path statistics with type info and FIRST SEGMENT type
    const char *path_type = dubins_path.is_simplified ? "SIMPLIFIED" : "DUBINS";
    const char *first_segment = "UNKNOWN";
    if (dubins_path.segments.size() > 0)
    {
      auto &seg = dubins_path.segments[0];
      if (seg.type == DubinsParking::PathSegment::LEFT)
        first_segment = "LEFT";
      else if (seg.type == DubinsParking::PathSegment::RIGHT)
        first_segment = "RIGHT";
      else
        first_segment = "STRAIGHT";
    }

    RCLCPP_INFO(this->get_logger(),
                "PATH: %s, len=%.2fm, eff=%.2f, dist=%.2fm, segments=%zu, FIRST=%s",
                path_type, dubins_path.total_length, dubins_path.path_efficiency,
                distance_to_center_pose, dubins_path.segments.size(), first_segment);

    // ============================
    // COLLISION CHECKING WITH TOLERANCE
    // ============================

    // Adaptive collision checking - more lenient when far, strict when close
    int collision_safety_margin = (distance_to_center_pose > 4.0) ? 1 : 2;
    int collision_threshold = (distance_to_center_pose > 4.0) ? 40 : 30;

    bool collision_detected = false;
    int collision_count = 0;
    double vehicle_yaw = current_theta;

    for (const auto &pose : dubins_path.poses)
    {
      // Transform pose to grid coordinates
      double dx = pose.x - this->occupancy_grid.info.origin.position.x;
      double dy = pose.y - this->occupancy_grid.info.origin.position.y;

      // Rotate into grid frame
      double rot_x = dx * std::cos(-vehicle_yaw) - dy * std::sin(-vehicle_yaw);
      double rot_y = dx * std::sin(-vehicle_yaw) + dy * std::cos(-vehicle_yaw);

      // Convert to grid indices
      int grid_x = static_cast<int>(rot_x / this->occupancy_grid.info.resolution);
      int grid_y = static_cast<int>(rot_y / this->occupancy_grid.info.resolution);

      // Check bounds
      if (grid_x >= 0 && grid_x < static_cast<int>(this->occupancy_grid.info.width) &&
          grid_y >= 0 && grid_y < static_cast<int>(this->occupancy_grid.info.height))
      {
        // Check for collision with adaptive safety margin
        for (int dx_check = -collision_safety_margin; dx_check <= collision_safety_margin; dx_check++)
        {
          for (int dy_check = -collision_safety_margin; dy_check <= collision_safety_margin; dy_check++)
          {
            int check_x = grid_x + dx_check;
            int check_y = grid_y + dy_check;

            if (check_x >= 0 && check_x < static_cast<int>(this->occupancy_grid.info.width) &&
                check_y >= 0 && check_y < static_cast<int>(this->occupancy_grid.info.height))
            {
              unsigned int idx = check_y * this->occupancy_grid.info.width + check_x;
              if (this->occupancy_grid.data[idx] > collision_threshold)
              {
                collision_count++;
                if (collision_count > 3)
                { // Allow a few collision points (noise tolerance)
                  collision_detected = true;
                  break;
                }
              }
            }
          }
          if (collision_detected)
            break;
        }
      }
      if (collision_detected)
        break;
    }

    if (collision_detected)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Dubins path has %d collision points! Using with caution.", collision_count);

      // Try with tighter radius if collision detected and far enough
      if (distance_to_center_pose > 3.0)
      {
        dubins_planner.setMinTurningRadius(safe_radius * 0.85);
        DubinsParking::DubinsPath retry_path = dubins_planner.planPath(start_pose, goal_pose);

        if (retry_path.valid)
        {
          RCLCPP_INFO(this->get_logger(), "Retry with tighter radius succeeded");
          dubins_path = retry_path;
          collision_detected = false;
        }
      }
    }

    // ============================
    // CONVERT TO ROS PATH MESSAGE WITH WAYPOINT PREPROCESSING
    // ============================

    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = this->get_clock()->now();

    // Waypoint preprocessing: Add intermediate waypoints for early curve initiation
    std::vector<DubinsParking::Pose> preprocessed_poses;

    for (size_t i = 0; i < dubins_path.poses.size(); ++i)
    {
      const auto &current = dubins_path.poses[i];
      preprocessed_poses.push_back(current);

      // Add extra waypoints at curve transitions for smoother controller tracking
      if (i > 0 && i < dubins_path.poses.size() - 1)
      {
        const auto &prev = dubins_path.poses[i - 1];
        const auto &next = dubins_path.poses[i + 1];

        // Detect heading change (curve)
        double heading_change = std::abs(current.theta - prev.theta);
        if (heading_change > 0.1)
        { // Significant curve
          // Insert intermediate waypoint
          DubinsParking::Pose intermediate;
          intermediate.x = (current.x + next.x) / 2.0;
          intermediate.y = (current.y + next.y) / 2.0;
          intermediate.theta = (current.theta + next.theta) / 2.0;
          preprocessed_poses.push_back(intermediate);
        }
      }
    }

    for (const auto &dubins_pose : preprocessed_poses)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = trajectory.header;
      pose.pose.position.x = dubins_pose.x;
      pose.pose.position.y = dubins_pose.y;
      pose.pose.position.z = 0.0;

      // Set orientation from Dubins theta
      tf2::Quaternion q;
      q.setRPY(0, 0, dubins_pose.theta);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      trajectory.poses.push_back(pose);
    }

    // Cache this path for future use
    last_valid_path = trajectory;
    has_valid_path = true;

    // ============================
    // VISUALIZATION
    // ============================

    // Create visualization markers for debugging
    visualization_msgs::msg::MarkerArray marker_array;

    // Path line marker
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = this->get_clock()->now();
    path_marker.ns = "dubins_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.1; // Line width
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0; // Green for Dubins path
    path_marker.color.b = 0.0;
    path_marker.color.a = 0.8;

    for (const auto &pose : trajectory.poses)
    {
      path_marker.points.push_back(pose.pose.position);
    }
    marker_array.markers.push_back(path_marker);

    // Start and goal markers
    visualization_msgs::msg::Marker start_marker;
    start_marker.header = path_marker.header;
    start_marker.ns = "dubins_path";
    start_marker.id = 1;
    start_marker.type = visualization_msgs::msg::Marker::ARROW;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.pose.position.x = current_x;
    start_marker.pose.position.y = current_y;
    start_marker.pose.position.z = 0.0;
    start_marker.pose.orientation = msg.pose.pose.orientation;
    start_marker.scale.x = 1.0;
    start_marker.scale.y = 0.3;
    start_marker.scale.z = 0.3;
    start_marker.color.r = 0.0;
    start_marker.color.g = 0.0;
    start_marker.color.b = 1.0; // Blue for start
    start_marker.color.a = 0.8;
    marker_array.markers.push_back(start_marker);

    visualization_msgs::msg::Marker goal_marker = start_marker;
    goal_marker.id = 2;
    goal_marker.pose.position.x = goal_x;
    goal_marker.pose.position.y = goal_y;
    goal_marker.pose.orientation = this->current_parking_goal_.pose.orientation;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0; // Red for goal
    marker_array.markers.push_back(goal_marker);

    // Publish visualization
    this->debug_paths_pub_->publish(marker_array);

    // Publish trajectory
    this->path_pub->publish(trajectory);
  }

  void TrajectoryPlanner::followCenterline(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    lanelet::BasicPoint2d current_point(msg.pose.pose.position.x, msg.pose.pose.position.y);

    size_t current_lane_idx = 0;
    size_t distance = std::numeric_limits<size_t>::max();

    for (size_t i = 0; i < this->path->lanes.size(); i++)
    {
      const lanelet::ConstLanelet current_lanelet = this->lanelet_map->laneletLayer.get(this->path->lanes[i]);
      size_t new_distance = lanelet::geometry::distance2d(current_lanelet, current_point);

      if (new_distance < distance)
      {
        distance = new_distance;
        current_lane_idx = i;
      }
    }

    lanelet::Lanelet current_lanelet = this->lanelet_map->laneletLayer.get(this->path->lanes[current_lane_idx]);

    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = this->now();

    bool found = false;
    std::vector<double> x_coords, y_coords;

    for (const auto &lane_id : this->path->lanes)
    {
      if (lane_id == current_lanelet.id())
        found = true; // Only follow the current lane
      if (!found)
        continue; // Skip lanes until we find the current lane

      lanelet::Lanelet lane = this->lanelet_map->laneletLayer.get(lane_id);
      for (const auto &point : lane.centerline())
      {
        x_coords.push_back(point.x());
        y_coords.push_back(point.y());
      }
    }

    if (x_coords.size() < 3)
    {
      RCLCPP_WARN(this->get_logger(), "Not enough points for interpolation.");
      return;
    }

    // Interpolate the path
    std::vector<double> distances;
    distances.push_back(0.0);
    for (size_t i = 1; i < x_coords.size(); ++i)
    {
      double d = std::hypot(x_coords[i] - x_coords[i - 1], y_coords[i] - y_coords[i - 1]);
      distances.push_back(distances.back() + d);
    }

    // Check if path has sufficient length
    if (distances.back() < 1e-6)
    {
      RCLCPP_ERROR(this->get_logger(), "Path has zero or near-zero length, cannot interpolate!");
      return;
    }

    std::vector<double> t;
    std::vector<double> x_filtered;
    std::vector<double> y_filtered;
    for (size_t i = 0; i < distances.size(); ++i)
    {
      double t_val = distances[i] / distances.back();
      // Only add point if it's the first or if it increases the parameter
      if (t.empty() || t_val > t.back() + 1e-9)
      {
        t.push_back(t_val);
        x_filtered.push_back(x_coords[i]);
        y_filtered.push_back(y_coords[i]);
      }
    }

    // Need at least 2 points for spline
    if (t.size() < 2)
    {
      RCLCPP_ERROR(this->get_logger(), "Not enough unique points for spline interpolation!");
      return;
    }

    tk::spline spline_x;
    tk::spline spline_y;
    spline_x.set_points(t, x_filtered);
    spline_y.set_points(t, y_filtered);

    // Increase resolution of interpolated path
    const int resolution = x_coords.size() * 5;
    for (int i = 0; i < resolution; ++i)
    {
      double ti = static_cast<double>(i) / (resolution - 1);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = spline_x(ti);
      pose.pose.position.y = spline_y(ti);
      pose.pose.position.z = 0.0;

      // Calculate orientation (yaw) using the derivative of the spline
      double dx, dy;
      if (i < resolution - 1)
      {
        double tip = static_cast<double>(i + 1) / (resolution - 1);
        dx = spline_x(tip) - spline_x(ti);
        dy = spline_y(tip) - spline_y(ti);
      }
      else if (i > 0)
      {
        double tim = static_cast<double>(i - 1) / (resolution - 1);
        dx = spline_x(ti) - spline_x(tim);
        dy = spline_y(ti) - spline_y(tim);
      }
      else
      {
        dx = 1.0;
        dy = 0.0;
      }
      double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      trajectory.poses.push_back(pose);
    }

    this->path_pub->publish(trajectory);
  }

  void TrajectoryPlanner::pathCallback(const gae_msgs::msg::GaePathIds &path_ids)
  {
    this->path = std::make_shared<gae_msgs::msg::GaePathIds>(path_ids);
    // RCLCPP_INFO(this->get_logger(), "Path received!");
  }

  void TrajectoryPlanner::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid grid)
  {
    this->occupancy_grid = grid;
  }

  void TrajectoryPlanner::readLaneletMap()
  {
    lanelet::Origin origin({this->origin_x, this->origin_y, 0.0});

    lanelet::projection::UtmProjector projector = lanelet::projection::UtmProjector(origin);

    this->lanelet_map = lanelet::load(this->osm_path, projector);

    if (this->lanelet_map == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load lanelet map");
      rclcpp::shutdown(); // BE CAREFUL FOR CONTAINERIZED APPLICATIONS
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Lanelet map is loaded successfully!");
    }
  }

  std::vector<double> TrajectoryPlanner::movingAverage(const std::vector<double> &data, int windowSize)
  {
    std::vector<double> smoothed;
    int n = data.size();
    int halfWindow = windowSize / 2;

    for (int i = 0; i < n; i++)
    {
      double sum = 0.0;
      int count = 0;
      // pencereyi verinin başından ve sonundan taşmayacak şekilde ayarla
      for (int j = std::max(0, i - halfWindow); j < std::min(n, i + halfWindow + 1); j++)
      {
        sum += data[j];
        count++;
      }
      smoothed.push_back(sum / count);
    }
    return smoothed;
  }
} // namespace itusct

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(itusct::TrajectoryPlanner)