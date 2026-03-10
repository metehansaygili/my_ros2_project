#ifndef PLANNER__TRAJECTORY_PLANNER_HPP_
#define PLANNER__TRAJECTORY_PLANNER_HPP_

// Standard library includes
#include <string>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

// Lanelet2 includes
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

// ROS2 message includes
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

// For visualization
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Custom message includes
#include <gae_msgs/msg/gae_path_ids.hpp>
#include "gae_msgs/msg/gae_telemetry.hpp"

// TF2 includes
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

// Local includes
#include "AStar.hpp"
#include "VirdultPlanner.hpp"
#include "dubins.hpp"
#include "utils.hpp"
#include "spline.h"

namespace itusct
{
    class TrajectoryPlanner : public rclcpp::Node
    {
    public:
        TrajectoryPlanner(const rclcpp::NodeOptions &options);

    private:
        // ========== Configuration Parameters ==========
        std::string osm_path;
        bool follow_centerline;
        double curvature_contributing_factor;
        double centerline_contributing_factor;
        int clearance;
        int ensuring_factor;
        bool camera_mode;
        std::string camera_path_topic;
        double current_speed_;
        double stop_a_star_planning_in_parking_sector_threshold;
        double stopping_distance_to_parking_spot;
        double lattice_planner_horizon;
        double max_speed_;
        double min_speed_;
        double lattice_max_horizon_;
        double lattice_min_horizon_;
        // ========== Map and Coordinate Data ==========
        lanelet::LaneletMapPtr lanelet_map;
        double origin_x;
        double origin_y;

        // ========== Current State Data ==========
        geometry_msgs::msg::Point current_position;
        gae_msgs::msg::GaePathIds::SharedPtr path;
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        nav_msgs::msg::Path latest_camera_path_;

        // ========== Parking Goal Data ==========
        geometry_msgs::msg::PoseStamped current_parking_goal_;
        bool has_parking_goal_{false};
        bool no_need_replan = false;

        // ========== TF2 Components ==========
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // ========== ROS2 Subscribers ==========
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub;
        rclcpp::Subscription<gae_msgs::msg::GaePathIds>::SharedPtr path_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_controller_sub;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_topic_sub;
        rclcpp::Subscription<gae_msgs::msg::GaeTelemetry>::SharedPtr telemetry_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr parking_goal_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr camera_path_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub;

        // ========== ROS2 Publishers ==========
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_paths_pub_;

        // ========== Helper Functions ==========
        void readLaneletMap();
        void followCenterline(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
        std::vector<double> movingAverage(const std::vector<double> &data, int windowSize);

        // ========== ROS2 Callback Functions ==========
        void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);
        void pathCallback(const gae_msgs::msg::GaePathIds &path_ids);
        void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid grid);
        void switchControllerCallback(const std_msgs::msg::Int32 &msg);
        void telemetryCallback(const gae_msgs::msg::GaeTelemetry::SharedPtr msg);
        void parkingGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void cameraPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

        // ========== State Machine Functions ==========
        planner::State current_state_ = planner::State::CENTERLINETRACK;

        // State Handlers
        void runCenterlineState(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
        void runLatticeState(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
        void runAStarState(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
        void runParkingAStarState(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
    };
} // namespace itusct

#endif // PLANNER__TRAJECTORY_PLANNER_HPP_