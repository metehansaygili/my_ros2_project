#ifndef PLANNER__GLOBAL_PLANNER_HPP_
#define PLANNER__GLOBAL_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Polygon.h> 
#include <lanelet2_core/primitives/Point.h>  
#include <lanelet2_core/geometry/Polygon.h> 
#include <lanelet2_core/geometry/Lanelet.h> 

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <gae_msgs/msg/gae_bottom_points_array.hpp>
#include <gae_msgs/msg/gae_bottom_points.hpp>
#include <gae_msgs/msg/gae_cam_shell_detection.hpp>
#include <gae_msgs/msg/gae_shell_polygon.hpp>
#include <gae_msgs/msg/gae_cam_detection_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <string>
#include <chrono>

namespace itusct{
    class GlobalPlanner : public rclcpp::Node
    {
    public:
        GlobalPlanner(const rclcpp::NodeOptions & options);
    private:
        // functions
        void readLaneletMap();
        void readIds();

        // variables
        lanelet::LaneletMapPtr lanelet_map;
        double origin_x;
        double origin_y;
        std::string osm_path;
        std::string station_name;
        std::string park_name;
        bool rviz_pose;
        bool is_park{false};
        bool has_target_{false};
        bool camera_mode{false};
        double PARKING_LOCK_DISTANCE;
        double min_confidence_to_consider_free_parking_spot_detection;
        int REQUIRED_STABLE_COUNT;


        lanelet::Id parking_lot_entry_id; 
        lanelet::Id first_goal_id; 

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::vector<size_t> station_ids;
        std::vector<size_t> park_ids;

        // ros related functions
        void rviz_goal_pose_callback(const geometry_msgs::msg::PoseStamped & msg);
        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
        void cluster_callback(const gae_msgs::msg::GaeBottomPointsArray & msg);
        void parking_pose_callback(const gae_msgs::msg::GaeCamShellDetection::SharedPtr msg);
        void yolo_callback(const gae_msgs::msg::GaeCamDetectionArray::SharedPtr msg);

        // ros related variables
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub; 
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr goal_pub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub; 
        rclcpp::Subscription<gae_msgs::msg::GaeBottomPointsArray>::SharedPtr cluster_sub; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_limit_pub; 
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr parking_center_point_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_park_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_id_pub_;
            
        geometry_msgs::msg::Point current_position; 
        geometry_msgs::msg::PoseStamped last_valid_parking_pose_;
        geometry_msgs::msg::PoseStamped out_msg;
        
        
        // Parking specific variables
        rclcpp::Subscription<gae_msgs::msg::GaeCamShellDetection>::SharedPtr parking_pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr parking_relay_pub_;
        gae_msgs::msg::GaeBottomPointsArray current_clusters;

        // Stop sign detection variables
        rclcpp::Subscription<gae_msgs::msg::GaeCamDetectionArray>::SharedPtr yolo_sub_;
        int stop_sign_counter_{0};
        bool stop_sign_cooldown_{false};  // true = ignore detections during 30s wait
        rclcpp::TimerBase::SharedPtr stop_sign_timer_;
    };
} // namespace itusct

#endif  // PLANNER__GLOBAL_PLANNER_HPP_