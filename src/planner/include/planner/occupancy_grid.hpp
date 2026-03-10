// TODO: Remove unnecessary variables.
#ifndef PLANNER__OCCUPANCY_GRID_HPP_
#define PLANNER__OCCUPANCY_GRID_HPP_

// ============================================================================
// SYSTEM INCLUDES
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// ============================================================================
// ROS2 INCLUDES
// ============================================================================
#include <rclcpp/rclcpp.hpp>

// ROS2 Messages
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>

// Custom Messages
#include <gae_msgs/msg/gae_bottom_points_array.hpp>
#include <gae_msgs/msg/gae_path_ids.hpp>

// TF2 Libraries
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ============================================================================
// THIRD-PARTY INCLUDES
// ============================================================================
// Lanelet2
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Clipper
#include "clipper2/clipper.h"

// ============================================================================
// DEBUG INCLUDES (TODO: Remove after debugging)
// ============================================================================
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace itusct
{

    class OccupancyGrid : public rclcpp::Node
    {
    public:
        // ========================================================================
        // CONSTRUCTOR
        // ========================================================================
        OccupancyGrid(const rclcpp::NodeOptions &options);

    private:
        // ========================================================================
        // GRID CONFIGURATION
        // ========================================================================
        double grid_resolution_; // Resolution of the occupancy grid in meters per cell
        int grid_size_;          // Number of cells in one dimension of the grid
        bool occupancy_ready;    // Flag to check if the occupancy grid is ready
        bool debug_mode;         // Flag to enable debug mode

        // ========================================================================
        // TRANSFORM HANDLING
        // ========================================================================
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // ========================================================================
        // MAP AND VEHICLE CONFIGURATION
        // ========================================================================
        std::string osm_path; // Path to the OSM file
        double origin_x;      // Grid origin X coordinate
        double origin_y;      // Grid origin Y coordinate

        double vehicle_width;  // Vehicle width in meters
        double vehicle_length; // Vehicle length in meters
        double safety_margin;  // Safety margin in meters

        // Obstacle inflation parameters
        double base_obstacle_radius_;        // Base obstacle size in meters
        double obstacle_core_radius_;        // Obstacle core radius in pixels
        double lateral_clearance_factor_;    // Dynamic obstacle lateral clearance multiplier
        double prediction_time_min_;         // Minimum prediction time in seconds
        double prediction_time_max_;         // Maximum prediction time in seconds
        double max_prediction_distance_;     // Maximum longitudinal prediction in meters

        lanelet::LaneletMapPtr lanelet_map_; // Pointer to the lanelet map
        
        // Obstacle tracking structures
        struct ObstacleMeasurement
        {
            double x;
            double y;
            rclcpp::Time stamp;
        };

        struct TrackedObstacle
        {
            double x_base;          // Position in base_link frame
            double y_base;
            double vx;              // Velocity estimate
            double vy;
            double probability;     // Existence probability [0, 1]
            double log_odds;        // Log-odds representation for Bayesian update
            rclcpp::Time last_seen;
            rclcpp::Time created_at;
            int id;
            bool is_static;         // Static vs dynamic classification
        };

        std::vector<ObstacleMeasurement> current_measurements_;
        std::vector<TrackedObstacle> tracked_obstacles_;
        int next_obstacle_id_ = 0;
        
        // Tracking parameters
        double association_distance_ = 2.0;       // meters
        double prob_hit_ = 0.7;                   // P(detection | obstacle exists)
        double prob_miss_ = 0.3;                  // P(no detection | obstacle exists)  
        double prob_threshold_render_ = 0.4;      // Minimum probability to render
        double prob_threshold_delete_ = 0.1;      // Minimum probability to keep
        double alpha_position_ = 0.3;             // smoothing factor for position
        double alpha_velocity_ = 0.5;             // smoothing factor for velocity
        double static_velocity_threshold_ = 0.5;  // m/s - below this considered static
        double prediction_decay_rate_ = 0.95;     // Probability decay per second during occlusion
        
        void updateTracks(const std::vector<ObstacleMeasurement>& measurements);
        double logOdds(double probability) const;
        double probability(double log_odds) const;


        // ========================================================================
        // CURRENT STATE
        // ========================================================================
        geometry_msgs::msg::Pose pose;   // Vehicle's current pose
        gae_msgs::msg::GaePathIds path_; // Path IDs for the vehicle to follow

        // ========================================================================
        // ROS2 PUBLISHERS
        bool camera_mode_;

        // ROS2 publishers
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lanelet_marker_pub_; // Debug
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_obstacles_pub_; // Debug

        // ========================================================================
        // ROS2 SUBSCRIBERS
        // ========================================================================

        // Subscribers
        message_filters::Subscriber<gae_msgs::msg::GaeBottomPointsArray> cluster_sub_filt_;
        message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> pose_sub_filt_;

        // Sync
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            gae_msgs::msg::GaeBottomPointsArray,
            geometry_msgs::msg::PoseWithCovarianceStamped>;

        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        rclcpp::Subscription<gae_msgs::msg::GaeBottomPointsArray>::SharedPtr cluster_sub_;
        rclcpp::Subscription<gae_msgs::msg::GaePathIds>::SharedPtr path_sub;

        // ========================================================================
        // HELPER FUNCTIONS
        // ========================================================================
        void readLaneletMap();
        std::optional<cv::Point> transform_point(const lanelet::ConstPoint3d &pt) const;

        // ========================================================================
        // ROS2 CALLBACK FUNCTIONS
        // ========================================================================
        void localization_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
        void clustersCallback(const gae_msgs::msg::GaeBottomPointsArray &msg);
        void pathCallback(const gae_msgs::msg::GaePathIds &msg);
        void syncedCallback(
            const gae_msgs::msg::GaeBottomPointsArray::ConstSharedPtr clusters_msg,
            const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg);
    };

} // namespace itusct

#endif // PLANNER__OCCUPANCY_GRID_HPP_