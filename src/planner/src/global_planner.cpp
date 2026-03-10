#include "planner/global_planner.hpp"

namespace itusct
{
  GlobalPlanner::GlobalPlanner(const rclcpp::NodeOptions &options)
      : Node("global_planner_exe", options),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "global_planner is initialized!");

    this->declare_parameter<double>("origin_pose._x", 0.0);
    this->declare_parameter<double>("origin_pose._y", 0.0);
    this->declare_parameter<std::string>("osm_path", "./map.osm");
    this->declare_parameter<std::string>("station_name", "station");
    this->declare_parameter<std::string>("park_name", "park");
    this->declare_parameter<bool>("rviz_pose", false);

    this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
    this->declare_parameter<std::string>("rviz_goal_topic", "rviz_goal_pose");
    this->declare_parameter<std::string>("pose_topic", "pose");
    this->declare_parameter<std::string>("cluster_topic", "bottom_points");
    this->declare_parameter<lanelet::Id>("first_goal_id", 0);
    this->declare_parameter<lanelet::Id>("parking_lot_entry_id", 0);
    this->declare_parameter<double>("min_confidence_to_consider_free_parking_spot_detection", 0.6);
    this->declare_parameter<double>("parking_lock_distance", 3.0);
    this->declare_parameter<int>("required_stable_count", 3);
    this->is_park = false;

    this->origin_x = this->get_parameter("origin_pose._x").as_double();
    this->origin_y = this->get_parameter("origin_pose._y").as_double();
    this->osm_path = this->get_parameter("osm_path").as_string();
    this->station_name = this->get_parameter("station_name").as_string();
    this->park_name = this->get_parameter("park_name").as_string();
    this->rviz_pose = this->get_parameter("rviz_pose").as_bool();

    std::string RVIZ_GOAL_TOPIC = this->get_parameter("rviz_goal_topic").as_string();
    std::string GOAL_POSE_TOPIC = this->get_parameter("goal_pose_topic").as_string();
    std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();
    std::string CLUSTER_TOPIC = this->get_parameter("cluster_topic").as_string();

    this->first_goal_id = this->get_parameter("first_goal_id").as_int();
    this->parking_lot_entry_id = this->get_parameter("parking_lot_entry_id").as_int();
    this->min_confidence_to_consider_free_parking_spot_detection = this->get_parameter("min_confidence_to_consider_free_parking_spot_detection").as_double();
    this->PARKING_LOCK_DISTANCE = this->get_parameter("parking_lock_distance").as_double();
    this->REQUIRED_STABLE_COUNT = this->get_parameter("required_stable_count").as_int();
    this->readLaneletMap();
    this->readIds();

    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "origin_pose._x: %f", this->origin_x);
    RCLCPP_INFO(this->get_logger(), "origin_pose._y: %f", this->origin_y);
    RCLCPP_INFO(this->get_logger(), "osm_path: %s", this->osm_path.c_str());
    RCLCPP_INFO(this->get_logger(), "station_name: %s", this->station_name.c_str());
    RCLCPP_INFO(this->get_logger(), "park_name: %s", this->park_name.c_str());
    RCLCPP_INFO(this->get_logger(), "rviz_pose: %s", this->rviz_pose ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "goal_pose_topic: %s", GOAL_POSE_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "rviz_goal_topic: %s", RVIZ_GOAL_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "pose_topic: %s", POSE_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "cluster_topic: %s", CLUSTER_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "first_goal_id: %d", this->first_goal_id);
    RCLCPP_INFO(this->get_logger(), "parking_lot_entry_id: %d", this->parking_lot_entry_id);
    RCLCPP_INFO(this->get_logger(), "min_confidence_to_consider_free_parking_spot_detection: %f", this->min_confidence_to_consider_free_parking_spot_detection);
    RCLCPP_INFO(this->get_logger(), "parking_lock_distance: %f", this->PARKING_LOCK_DISTANCE);
    RCLCPP_INFO(this->get_logger(), "required_stable_count: %d", this->REQUIRED_STABLE_COUNT);

    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.transient_local().reliable();
    this->goal_pub = this->create_publisher<std_msgs::msg::Int32>(GOAL_POSE_TOPIC, qos_profile);
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(POSE_TOPIC, 10, std::bind(&GlobalPlanner::pose_callback, this, std::placeholders::_1));
    this->speed_limit_pub = this->create_publisher<std_msgs::msg::Float64>("/speed_limit", 10);
    this->parking_center_point_pub = this->create_publisher<visualization_msgs::msg::Marker>("/parking_center_point_marker", 10);
    this->current_id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/debug/current_lanelet_id", 10);
    this->is_park_pub_ = this->create_publisher<std_msgs::msg::Bool>("/is_park", 10);

    // Initialize Parking Subscribers and Publishers HERE inside the constructor
    this->parking_pose_sub_ = this->create_subscription<gae_msgs::msg::GaeCamShellDetection>(
        "/parking_area/center_pose",
        10,
        std::bind(&GlobalPlanner::parking_pose_callback, this, std::placeholders::_1));

    this->parking_relay_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/global_planner/final_parking_pose",
        10);

    // Subscribe to YOLO detections for stop sign based parking trigger
    this->yolo_sub_ = this->create_subscription<gae_msgs::msg::GaeCamDetectionArray>(
        "yolo_detections",
        10,
        std::bind(&GlobalPlanner::yolo_callback, this, std::placeholders::_1));

    if (this->rviz_pose)
    {
      this->rviz_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(RVIZ_GOAL_TOPIC, 10, std::bind(&GlobalPlanner::rviz_goal_pose_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "RVIZ goal pose is enabled");
    }
    else
    {
      this->cluster_sub = this->create_subscription<gae_msgs::msg::GaeBottomPointsArray>(CLUSTER_TOPIC, 10, std::bind(&GlobalPlanner::cluster_callback, this, std::placeholders::_1));
      std_msgs::msg::Int32 goal_id_msg;
      goal_id_msg.data = this->first_goal_id;
      this->goal_pub->publish(goal_id_msg);
      RCLCPP_INFO(this->get_logger(), "RVIZ goal pose is disabled, publishing first goal ID: %ld", this->first_goal_id);
    }
  }

  void GlobalPlanner::readLaneletMap()
  {
    // Create a projector using your origin
    lanelet::Origin origin({this->origin_x, this->origin_y, 0.0});

    lanelet::projection::UtmProjector projector = lanelet::projection::UtmProjector(origin);

    this->lanelet_map = lanelet::load(this->osm_path, projector);

    if (this->lanelet_map == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load lanelet map");
      rclcpp::shutdown();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Lanelet map is loaded successfully!");
    }
  }

  void GlobalPlanner::readIds()
  {
    for (const auto &lanelet : this->lanelet_map->laneletLayer)
    {
      if (lanelet.hasAttribute(this->station_name))
      {
        this->station_ids.push_back(lanelet.id());
      }
      else if (lanelet.hasAttribute(this->park_name))
      {
        this->park_ids.push_back(lanelet.id());
      }
    }
  }

  void GlobalPlanner::rviz_goal_pose_callback(const geometry_msgs::msg::PoseStamped &msg)
  {
    if (!this->rviz_pose)
    {
      RCLCPP_WARN(this->get_logger(), "RVIZ goal pose is not enabled!");
      return;
    }

    const lanelet::BasicPoint2d goal_point(msg.pose.position.x, msg.pose.position.y);

    RCLCPP_INFO(this->get_logger(), "Goal pose received: x: %f, y: %f", goal_point.x(), goal_point.y());

    std_msgs::msg::Int32 goal_id_msg;
    goal_id_msg.data = this->lanelet_map->laneletLayer.nearest(goal_point, 1)[0].id();

    this->goal_pub->publish(goal_id_msg);
  }

  void GlobalPlanner::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    this->current_position.x = msg.pose.pose.position.x;
    this->current_position.y = msg.pose.pose.position.y;

    // Keep finding the lanelet for other things (like speed limits), but we won't rely on it for parking trigger
    lanelet::BasicPoint2d current_point(this->current_position.x, this->current_position.y);
    auto current_lane = this->lanelet_map->laneletLayer.nearest(current_point, 1)[0];

    // Debug Publisher (Keep this)
    std_msgs::msg::Int32 id_msg;
    id_msg.data = current_lane.id();
    this->current_id_pub_->publish(id_msg);

    // Always publish current is_park state
    std_msgs::msg::Bool park_msg;
    park_msg.data = this->is_park;
    this->is_park_pub_->publish(park_msg);

    if (current_lane.hasAttribute("speed_limit"))
    {
      auto a = current_lane.attributes()["speed_limit"].asDouble();
      if (a.has_value())
      {
        double speed_limit = a.value();
        std_msgs::msg::Float64 speed_limit_msg;
        speed_limit_msg.data = speed_limit;
        this->speed_limit_pub->publish(speed_limit_msg);
      }
    }
  }

  void GlobalPlanner::yolo_callback(const gae_msgs::msg::GaeCamDetectionArray::SharedPtr msg)
  {
    // Always publish current is_park state
    std_msgs::msg::Bool park_msg;
    park_msg.data = this->is_park;
    this->is_park_pub_->publish(park_msg);

    // Already triggered parking or currently in cooldown – ignore
    if (is_park || stop_sign_cooldown_)
    {
      return;
    }

    bool stop_seen = false;
    for (const auto &detection : msg->detections)
    {
      if (detection.label == gae_msgs::msg::GaeCamDetection::STOP)
      {
        stop_seen = true;
        break;
      }
    }

    if (!stop_seen)
    {
      return;
    }

    // Stop sign detected
    stop_sign_counter_++;
    RCLCPP_WARN(this->get_logger(),
                "Stop sign detected! Counter: %d / 2", stop_sign_counter_);

    if (stop_sign_counter_ >= 2)
    {
      is_park = true;
      RCLCPP_WARN(this->get_logger(),
                  "Stop sign seen twice – triggering parking mode!");
      return;
    }

    // First detection: enter 30-second cooldown, ignore YOLO until timer fires
    stop_sign_cooldown_ = true;
    RCLCPP_INFO(this->get_logger(),
                "First stop sign detected. Waiting 30 seconds before re-listening...");

    stop_sign_timer_ = this->create_wall_timer(
        std::chrono::seconds(30),
        [this]() {
          stop_sign_timer_->cancel();  // one-shot
          stop_sign_cooldown_ = false;
          RCLCPP_INFO(this->get_logger(),
                      "30-second cooldown over. Listening for stop sign again...");
        });
  }

  void GlobalPlanner::parking_pose_callback(const gae_msgs::msg::GaeCamShellDetection::SharedPtr msg)
  {
    // Publish a marker for visualization
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "parking_center";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = msg->center_pose.pose.position.x;
      marker.pose.position.y = msg->center_pose.pose.position.y;
      marker.pose.position.z = msg->center_pose.pose.position.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      this->parking_center_point_pub->publish(marker);
    }

    // Check if parking is not triggered
    if (!is_park)
    {
      // Reset için
      has_target_ = false;
      return;
    }

    // **ÖNEMLİ: Hedef kilitliyse artık yeni mesajları işleme!**
    if (has_target_)
    {
      // Sadece kilitli hedefi yayınla, yeni park alanlarına bakma
      geometry_msgs::msg::PoseStamped out_msg = this->last_valid_parking_pose_;
      out_msg.header.stamp = this->get_clock()->now();
      this->parking_relay_pub_->publish(out_msg);

      RCLCPP_DEBUG(this->get_logger(), "Target locked - ignoring new detections");
      return; // Fonksiyondan çık!
    }

    // Calculate distance to the parking center
    double dx = msg->center_pose.pose.position.x - current_position.x;
    double dy = msg->center_pose.pose.position.y - current_position.y;
    double distance = std::hypot(dx, dy);

    static double max_confidence = 0.0;
    static int stable_count = 0;

    double current_confidence = msg->confidence;

    if (current_confidence < this->min_confidence_to_consider_free_parking_spot_detection)
    {
      return;
    }

    if (current_confidence > max_confidence)
    {
      max_confidence = current_confidence;
      stable_count = 1;
      this->last_valid_parking_pose_ = msg->center_pose;

      RCLCPP_INFO(this->get_logger(),
                  "Better parking spot found. Distance: %.2f m, Confidence: %.2f",
                  distance, current_confidence);
    }
    else if (std::abs(current_confidence - max_confidence) < 0.05) // Aynı hedef
    {
      stable_count++;
    }

    if (distance < this->PARKING_LOCK_DISTANCE && stable_count >= this->REQUIRED_STABLE_COUNT)
    {
      has_target_ = true;
      this->last_valid_parking_pose_ = msg->center_pose;

      RCLCPP_WARN(this->get_logger(),
                  " PARKING TARGET LOCKED! Distance: %.2f m, Confidence: %.2f - No longer accepting new targets!",
                  distance, current_confidence);
    }

    // Publish the parking pose
    geometry_msgs::msg::PoseStamped out_msg = this->last_valid_parking_pose_;
    out_msg.header.stamp = this->get_clock()->now();
    this->parking_relay_pub_->publish(out_msg);
  }

  void GlobalPlanner::cluster_callback(const gae_msgs::msg::GaeBottomPointsArray &msg)
  {
    const lanelet::BasicPoint2d current_point(this->current_position.x, this->current_position.y);

    this->current_clusters = msg;

    int currentId = this->lanelet_map->laneletLayer.nearest(current_point, 1)[0].id();

    if (currentId != this->parking_lot_entry_id)
    {
      return;
    }

    std::map<int, bool> isAvailable;
    for (const auto &id : this->park_ids)
    {
      isAvailable[id] = true;
    }

    std::vector<lanelet::BasicPoint2d> occupied_points;

    for (const auto &cluster : msg.clusters)
    {
      int num = 0;
      float total_x = 0.0;
      float total_y = 0.0;

      for (const auto &point : cluster.bottom_points_array)
      {
        geometry_msgs::msg::PointStamped cluster_point;
        cluster_point.header.frame_id = msg.header.frame_id;
        cluster_point.point.x = point.x;
        cluster_point.point.y = point.y;
        cluster_point.point.z = point.z;

        geometry_msgs::msg::PointStamped transformed_point;
        try
        {
          transformed_point = tf_buffer_.transform(cluster_point, "map");
          total_x += transformed_point.point.x;
          total_y += transformed_point.point.y;
        }
        catch (const tf2::TransformException &ex)
        {
          continue;
        }
        num++;
      }

      float avg_x = total_x / num;
      float avg_y = total_y / num;

      lanelet::BasicPoint2d avg_point;
      avg_point.x() = avg_x;
      avg_point.y() = avg_y;

      occupied_points.push_back(avg_point);
    }

    for (const auto &id : this->park_ids)
    {
      if (!isAvailable[id])
      {
        continue;
      }
      auto it = this->lanelet_map->laneletLayer.find(id);
      auto lanelet = *it;

      for (const auto &avg_point : occupied_points)
      {
        if (lanelet::geometry::inside(lanelet, avg_point) && isAvailable[id])
        {
          isAvailable[id] = false;
        }
        else if (!isAvailable[id])
        {
          isAvailable[id] = false;
        }
      }
    }
    for (const auto &[id, available] : isAvailable)
    {
      if (available) // it disrupts planner if no available parking spot
      {
        std_msgs::msg::Int32 goal_id_msg;
        goal_id_msg.data = id;
        this->goal_pub->publish(goal_id_msg);
        return;
      }
    }
  }
} // end of namespace itusct

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(itusct::GlobalPlanner)