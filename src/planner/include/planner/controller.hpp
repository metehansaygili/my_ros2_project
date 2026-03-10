#ifndef PLANNER__CONTROLLER_HPP_
#define PLANNER__CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gae_msgs/msg/gae_control_cmd.hpp>
#include <gae_msgs/msg/gae_telemetry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <nav_msgs/msg/path.hpp>
#include <utils.hpp>
#include "PIDController.hpp"
#include "PurePursuitController.hpp"
#include "MPCController.hpp"
#include "StanleyController.hpp"

#include <string>
#include <deque>
#include <cmath>
#include <limits>

namespace itusct
{
    class Controller : public rclcpp::Node
    {
    public:
        Controller(const rclcpp::NodeOptions &options);

    private:
        // ros related functions
        void trajectoryCallback(const nav_msgs::msg::Path &path);
        void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);
        void distanceToStopCallback(const std_msgs::msg::Float64 &msg);
        void telemetryCallback(const gae_msgs::msg::GaeTelemetry &msg);
        void speedLimitCallback(const std_msgs::msg::Float64 &msg);
        void vref_generator();
        void stopCallback(const std_msgs::msg::Bool &msg);
        void switchControllerCallback(const std_msgs::msg::Bool &msg);
        void speedCallback(const geometry_msgs::msg::TwistWithCovarianceStamped &msg);
        double speedController(double v_current, double v_ref);
        double computeDynamicDerivative(const nav_msgs::msg::Path &path_transformed);
        void applyThrottleLimit(double pid_output, gae_msgs::msg::GaeControlCmd &vehicle_cmd);
        void state_sub_callback(const std_msgs::msg::Int32 &msg);
        void imuCallback(const sensor_msgs::msg::Imu &msg); // IMU callback for yaw rate
        
        bool camera_mode;
        void controlLoop();
        rclcpp::TimerBase::SharedPtr control_timer_;

        // helper functions
        // double stddev_of_points(const std::vector<geometry_msgs::msg::PoseStamped>& pts, size_t start_index);
        // double compute_curvature_factor(double curvature_stddev);

        // MPC helper: Extract path segment within specified arc length from current position
        nav_msgs::msg::Path extractRelevantPath(
            const geometry_msgs::msg::Pose &current_pose,
            const nav_msgs::msg::Path &full_path,
            double lookahead_distance);

        // ros related variables
        rclcpp::Publisher<gae_msgs::msg::GaeControlCmd>::SharedPtr autonomous_vehicle_cmd_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr look_ahead_marker_pub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_to_stop_sub;
        rclcpp::Subscription<gae_msgs::msg::GaeTelemetry>::SharedPtr telemetry_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_controller_sub;
        rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr speed_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr transformed_path_pub;
        rclcpp::Publisher<gae_msgs::msg::GaeTelemetry>::SharedPtr telemetry_pub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_limit_sub;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_topic_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_trajectory_pub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // IMU for yaw rate
        // variables

        rclcpp::TimerBase::SharedPtr predict_timer_;
        bool apply_brake_distance_stop = false;
        double base_lookahead_distance;
        double sensitivity;
        double max_speed;
        double min_speed;
        double max_steering_angle;
        double min_lookahead_distance;
        double max_stdv;
        double brake_distance; // for 200 rpm it equals to 10 meters
        double wheelbase;
        double distance_to_stop;
        double current_speed;
        double current_speed_ms = 0.0;
        double base_max_speed;
        planner::State state = planner::State::CENTERLINETRACK;
        double max_acceleration; // Maximum acceleration in RPM/s
        double max_deceleration; // Maximum deceleration in RPM/s
        double prev_steering_error = 0.0;
        const size_t integral_error_window_size = 50;                                                   // Maximum size of the window
        std::deque<double> integral_error_window = std::deque<double>(integral_error_window_size, 0.0); // Window initialized with zeros
        double speed_limit;
        bool stop_topic_info = false;
        bool switch_info = false;
        PIDController pid_controller;
        std::unique_ptr<PurePursuitController> pure_pursuit_controller_;

        // MPC Controller
        bool use_mpc_;
        std::unique_ptr<itusct::MPCController> mpc_controller_;
        int mpc_horizon_;
        double mpc_dt_;
        double max_mpc_steering_rad_;        // ±27 degrees in radians
        double prev_mpc_steering_deg_ = 0.0; // For rate limiting
        double current_yaw_rate_ = 0.0;      // Yaw rate from IMU for MPC (rad/s)

        // Stanley Controller (for high-speed racing)
        bool use_stanley_;
        itusct::StanleyController stanley_controller_;

        double Kp;
        double Kd;
        double Ts;               // Sampling time in seconds # update Ts according to localization rate
        double Ukd_prev = 0.0;   // Previous derivative term
        double error_prev = 0.0; // Previous error
        double referenceSpeed = 0.0;

        // PID parameters for speed controller
        double Kp_speed;
        double Ki_speed;
        double Kd_speed;
        double Uki_prev_speed = 0.0;             // Previous integral term
        double Ukd_prev_speed = 0.0;             // Previous derivative term
        double error_prev_speed = 0.0;           // Previous error
        rclcpp::Time last_control_time_;         // Last time control command was sent
        double previous_throttle_request_ = 0.0; // Previous throttle command

        // Store the last time that the desired speed was set
        rclcpp::Time last_speed_update_time;
        rclcpp::Clock::SharedPtr clock_;
        double max_throttle_increase_rate_; // Maximum throttle increase rate (RPM per control loop)
        double max_throttle_decrease_rate_; // Maximum throttle decrease rate (RPM per control loop)
        double min_brake_threshold_;         // Minimum brake value to activate braking
        double mpc_brake_acceleration_threshold_; // Acceleration threshold to trigger MPC direct brake
        double mpc_brake_gain_;                   // Gain for converting negative acceleration to brake command
        double output_throttle;

        geometry_msgs::msg::Pose current_pose;
        double current_yaw;
        nav_msgs::msg::Path path;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
} // namespace itusct

#endif // PLANNER__CONTROLLER_HPP_