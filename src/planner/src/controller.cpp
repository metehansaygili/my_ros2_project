#include "controller.hpp"
#define WHEEL_RADIUS 0.45
namespace itusct
{

    Controller::Controller(const rclcpp::NodeOptions &options)
        : Node("controller_exe", options)
    {
        RCLCPP_INFO(this->get_logger(), "controller initialized!");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter<std::string>("trajectory_topic", "/path");
        this->declare_parameter<std::string>("pose_topic", "/pose");
        this->declare_parameter<std::string>("autonomous_cmd_topic", "/controller/vehicle_control");
        this->declare_parameter<std::string>("distance_to_stop_topic", "/mission_planner/stop_distance");
        this->declare_parameter<std::string>("telemetry_topic", "/vehicle/telemetry");
        this->declare_parameter<double>("base_lookahead_distance", 6.0);
        this->declare_parameter<double>("sensitivity", 5.5);
        this->declare_parameter<double>("max_speed", 100.0);
        this->declare_parameter<double>("min_speed", 40.0);
        this->declare_parameter<double>("max_steering_angle", 90.0);
        this->declare_parameter<double>("min_lookahead_distance", 3.5);
        this->declare_parameter<double>("brake_distance", 20.0);
        this->declare_parameter<double>("wheelbase", 1.5);
        this->declare_parameter<double>("max_acceleration", 20.0);
        this->declare_parameter<double>("max_deceleration", 30.0);
        this->declare_parameter<double>("Kp", 2.5);
        this->declare_parameter<double>("Kd", 0.05);
        this->declare_parameter<double>("Ts", 0.1);
        this->declare_parameter<double>("Kp_speed", 1.0);
        this->declare_parameter<double>("Ki_speed", 0.1);
        this->declare_parameter<double>("Kd_speed", 0.01);
        this->declare_parameter<std::string>("switch_controller_topic", "/switch_controller");

        // MPC Controller parameters
        this->declare_parameter<bool>("use_mpc", true);
        this->declare_parameter<int>("mpc_horizon", 20);
        this->declare_parameter<double>("mpc_dt", 0.1);
        // MPC Weights (configurable via YAML)
        this->declare_parameter<double>("mpc_w_cte", 50.0);
        this->declare_parameter<double>("mpc_w_epsi", 40.0);
        this->declare_parameter<double>("mpc_w_v", 1.0);
        this->declare_parameter<double>("mpc_w_delta", 10.0);
        this->declare_parameter<double>("mpc_w_ddelta", 1500.0);
        this->declare_parameter<double>("mpc_w_a", 1.0);
        this->declare_parameter<double>("mpc_w_da", 10.0);
        this->declare_parameter<double>("mpc_w_omega", 100.0);  // Yaw rate penalty for oscillation damping
        this->declare_parameter<double>("mpc_w_domega", 500.0); // Yaw acceleration penalty
        this->declare_parameter<std::string>("imu_topic", "/sensing/imu/imu_raw");

        this->declare_parameter<double>("max_throttle_increase_rate", 15.0);
        this->declare_parameter<double>("max_throttle_decrease_rate", 25.0);
        this->declare_parameter<double>("min_brake_threshold", 5.0);

        this->declare_parameter<double>("mpc_brake_acceleration_threshold", -0.8);
        this->declare_parameter<double>("mpc_brake_gain", 15.0);

        std::string TRAJECTORY_TOPIC = this->get_parameter("trajectory_topic").as_string();
        std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();
        std::string AUTONOMOUS_VEHICLE_CMD = this->get_parameter("autonomous_cmd_topic").as_string();
        std::string DISTANCE_TO_STOP_TOPIC = this->get_parameter("distance_to_stop_topic").as_string();
        std::string TELEMETRY_TOPIC = this->get_parameter("telemetry_topic").as_string();
        std::string SWITCH_CONTROLLER_TOPIC = this->get_parameter("switch_controller_topic").as_string();

        this->base_lookahead_distance = this->get_parameter("base_lookahead_distance").as_double();
        this->sensitivity = this->get_parameter("sensitivity").as_double();
        this->max_speed = this->get_parameter("max_speed").as_double();
        this->min_speed = this->get_parameter("min_speed").as_double();
        this->max_acceleration = this->get_parameter("max_acceleration").as_double();
        this->max_deceleration = this->get_parameter("max_deceleration").as_double();
        this->max_steering_angle = this->get_parameter("max_steering_angle").as_double();
        this->min_lookahead_distance = this->get_parameter("min_lookahead_distance").as_double();
        this->brake_distance = this->get_parameter("brake_distance").as_double();
        this->wheelbase = this->get_parameter("wheelbase").as_double();
        this->Kp = this->get_parameter("Kp").as_double();
        this->Kd = this->get_parameter("Kd").as_double();
        this->Ts = this->get_parameter("Ts").as_double();
        this->Kp_speed = this->get_parameter("Kp_speed").as_double();
        this->Ki_speed = this->get_parameter("Ki_speed").as_double();
        this->Kd_speed = this->get_parameter("Kd_speed").as_double();
        this->max_throttle_increase_rate_ = this->get_parameter("max_throttle_increase_rate").as_double();
        this->max_throttle_decrease_rate_ = this->get_parameter("max_throttle_decrease_rate").as_double();
        this->min_brake_threshold_ = this->get_parameter("min_brake_threshold").as_double();
        this->mpc_brake_acceleration_threshold_ = this->get_parameter("mpc_brake_acceleration_threshold").as_double();
        this->mpc_brake_gain_ = this->get_parameter("mpc_brake_gain").as_double();

        // Print all parameters
        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "  trajectory_topic: %s", TRAJECTORY_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "  pose_topic: %s", POSE_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "  autonomous_cmd_topic: %s", AUTONOMOUS_VEHICLE_CMD.c_str());
        RCLCPP_INFO(this->get_logger(), "  distance_to_stop_topic: %s", DISTANCE_TO_STOP_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "  telemetry_topic: %s", TELEMETRY_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "  base_lookahead_distance: %.2f", this->base_lookahead_distance);
        RCLCPP_INFO(this->get_logger(), "  sensitivity: %.2f", this->sensitivity);
        RCLCPP_INFO(this->get_logger(), "  max_speed: %.2f", this->max_speed);
        RCLCPP_INFO(this->get_logger(), "  min_speed: %.2f", this->min_speed);
        RCLCPP_INFO(this->get_logger(), "  max_steering_angle: %.2f", this->max_steering_angle);
        RCLCPP_INFO(this->get_logger(), "  min_lookahead_distance: %.2f", this->min_lookahead_distance);
        RCLCPP_INFO(this->get_logger(), "  brake_distance: %.2f", this->brake_distance);
        RCLCPP_INFO(this->get_logger(), "  wheelbase: %.2f", this->wheelbase);
        RCLCPP_INFO(this->get_logger(), "  max_acceleration: %.2f", this->max_acceleration);
        RCLCPP_INFO(this->get_logger(), "  max_deceleration: %.2f", this->max_deceleration);
        RCLCPP_INFO(this->get_logger(), "  Kp: %.2f", this->Kp);
        RCLCPP_INFO(this->get_logger(), "  Kd: %.2f", this->Kd);
        RCLCPP_INFO(this->get_logger(), "  Ts: %.2f", this->Ts);
        RCLCPP_INFO(this->get_logger(), "  Kp_speed: %.2f", this->Kp_speed);
        RCLCPP_INFO(this->get_logger(), "  Ki_speed: %.2f", this->Ki_speed);
        RCLCPP_INFO(this->get_logger(), "  Kd_speed: %.2f", this->Kd_speed);
        RCLCPP_INFO(this->get_logger(), "  max_throttle_increase_rate: %.2f", this->max_throttle_increase_rate_);
        RCLCPP_INFO(this->get_logger(), "  max_throttle_decrease_rate: %.2f", this->max_throttle_decrease_rate_);
        RCLCPP_INFO(this->get_logger(), "  min_brake_threshold: %.2f", this->min_brake_threshold_);
        RCLCPP_INFO(this->get_logger(), "  mpc_brake_acceleration_threshold: %.2f", this->mpc_brake_acceleration_threshold_);
        RCLCPP_INFO(this->get_logger(), "  mpc_brake_gain: %.2f", this->mpc_brake_gain_);

        this->output_throttle = 0.0;
        this->current_speed = 0.0;
        this->clock_ = this->get_clock();
        this->last_speed_update_time = this->clock_->now();
        this->distance_to_stop = std::numeric_limits<double>::max();
        this->base_max_speed = this->max_speed; // Store the base max speed to reset it later if needed

        this->autonomous_vehicle_cmd_pub = this->create_publisher<gae_msgs::msg::GaeControlCmd>(
            AUTONOMOUS_VEHICLE_CMD, 10);
        this->trajectory_sub = this->create_subscription<nav_msgs::msg::Path>(TRAJECTORY_TOPIC, 10, std::bind(&Controller::trajectoryCallback, this, std::placeholders::_1));
        this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(POSE_TOPIC, 10, std::bind(&Controller::poseCallback, this, std::placeholders::_1));
        this->look_ahead_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            "/look_ahead_marker", 10); // make topic name configurable with config file.
        this->distance_to_stop_sub = this->create_subscription<std_msgs::msg::Float64>(DISTANCE_TO_STOP_TOPIC, 10, std::bind(&Controller::distanceToStopCallback, this, std::placeholders::_1));
        this->telemetry_sub = this->create_subscription<gae_msgs::msg::GaeTelemetry>(TELEMETRY_TOPIC, 10, std::bind(&Controller::telemetryCallback, this, std::placeholders::_1));
        this->stop_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/stop", 10, std::bind(&Controller::stopCallback, this, std::placeholders::_1));
        this->switch_controller_sub = this->create_subscription<std_msgs::msg::Bool>(SWITCH_CONTROLLER_TOPIC, 10, std::bind(&Controller::switchControllerCallback, this, std::placeholders::_1));
        this->speed_sub = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/vehicle/cmd_vel_cov", 10, std::bind(&Controller::speedCallback, this, std::placeholders::_1));
        this->transformed_path_pub = this->create_publisher<nav_msgs::msg::Path>(
            "/transformed_path", 10);
        this->telemetry_pub_ = this->create_publisher<gae_msgs::msg::GaeTelemetry>(TELEMETRY_TOPIC, 10);
        this->state_topic_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/vehicle_state", 10, std::bind(&Controller::state_sub_callback, this, std::placeholders::_1));

        this->predicted_trajectory_pub = this->create_publisher<nav_msgs::msg::Path>("/predicted_trajectory", 10);

        this->last_control_time_ = this->clock_->now();

        // IMU subscription for yaw rate (oscillation damping)
        std::string IMU_TOPIC = this->get_parameter("imu_topic").as_string();
        this->imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, 10, std::bind(&Controller::imuCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "IMU subscription: %s", IMU_TOPIC.c_str());

        pid_controller.setGains(1.3, 0.05, 0.1);
        pid_controller.setDerivativeLimit(100.0); // Set derivative spike rejection limit

        // In your Controller constructor:
        pure_pursuit_controller_ = std::make_unique<PurePursuitController>(
            this->wheelbase,
            this->base_lookahead_distance,
            this->min_lookahead_distance,
            this->max_speed,
            this->sensitivity);

        // Initialize MPC Controller
        this->use_mpc_ = this->get_parameter("use_mpc").as_bool();
        this->mpc_horizon_ = this->get_parameter("mpc_horizon").as_int();
        this->mpc_dt_ = this->get_parameter("mpc_dt").as_double();
        this->max_mpc_steering_rad_ = 27.0 * M_PI / 180.0; // ±27 degrees in radians

        mpc_controller_ = std::make_unique<itusct::MPCController>(
            this->wheelbase,
            this->mpc_horizon_,
            this->mpc_dt_);

        // Apply MPC weights from config
        itusct::MPCWeights mpc_weights;
        mpc_weights.w_cte = this->get_parameter("mpc_w_cte").as_double();
        mpc_weights.w_epsi = this->get_parameter("mpc_w_epsi").as_double();
        mpc_weights.w_v = this->get_parameter("mpc_w_v").as_double();
        mpc_weights.w_delta = this->get_parameter("mpc_w_delta").as_double();
        mpc_weights.w_ddelta = this->get_parameter("mpc_w_ddelta").as_double();
        mpc_weights.w_a = this->get_parameter("mpc_w_a").as_double();
        mpc_weights.w_da = this->get_parameter("mpc_w_da").as_double();
        mpc_weights.w_omega = this->get_parameter("mpc_w_omega").as_double();
        mpc_weights.w_domega = this->get_parameter("mpc_w_domega").as_double();
        mpc_controller_->setWeights(mpc_weights);

        RCLCPP_INFO(this->get_logger(), "MPC Controller initialized: use_mpc=%s, horizon=%d, dt=%.2f",
                    this->use_mpc_ ? "true" : "false", this->mpc_horizon_, this->mpc_dt_);
        RCLCPP_INFO(this->get_logger(), "MPC Weights: cte=%.1f epsi=%.1f delta=%.1f ddelta=%.1f",
                    mpc_weights.w_cte, mpc_weights.w_epsi, mpc_weights.w_delta, mpc_weights.w_ddelta);

        // Initialize Stanley Controller
        this->declare_parameter<bool>("use_stanley", true);
        this->declare_parameter<double>("stanley_k_cte", 2.0);
        this->declare_parameter<double>("stanley_k_heading", 1.0);
        this->declare_parameter<double>("stanley_k_soft", 1.0);

        this->use_stanley_ = this->get_parameter("use_stanley").as_bool();
        itusct::StanleyController::StanleyParams stanley_params;
        stanley_params.k_cte = this->get_parameter("stanley_k_cte").as_double();
        stanley_params.k_heading = this->get_parameter("stanley_k_heading").as_double();
        stanley_params.k_soft = this->get_parameter("stanley_k_soft").as_double();
        stanley_params.max_steering = 27.0 * M_PI / 180.0; // 27 degrees in radians
        stanley_controller_.setParams(stanley_params);

        RCLCPP_INFO(this->get_logger(), "Stanley Controller initialized: use_stanley=%s, k_cte=%.1f, k_heading=%.1f",
                    this->use_stanley_ ? "true" : "false", stanley_params.k_cte, stanley_params.k_heading);

        this->declare_parameter<bool>("camera_mode", false);
        this->get_parameter("camera_mode", this->camera_mode);

        if (this->camera_mode)
        {
            this->control_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50), std::bind(&Controller::controlLoop, this));
            RCLCPP_INFO(this->get_logger(), "Camera Mode Active. Starting Control Timer (20Hz).");
        }

        RCLCPP_INFO(this->get_logger(), "Controller initialized successfully.");
    }

    void Controller::controlLoop()
    {
        if (!this->camera_mode)
            return;

        // Use Identity Pose for base_link control
        geometry_msgs::msg::PoseWithCovarianceStamped identity_pose;
        identity_pose.header.frame_id = "base_link";
        identity_pose.header.stamp = this->get_clock()->now();
        identity_pose.pose.pose.position.x = 0.0;
        identity_pose.pose.pose.position.y = 0.0;
        identity_pose.pose.pose.position.z = 0.0;
        identity_pose.pose.pose.orientation.w = 1.0;
        identity_pose.pose.pose.orientation.x = 0.0;
        identity_pose.pose.pose.orientation.y = 0.0;
        identity_pose.pose.pose.orientation.z = 0.0;

        // Trigger the standard control logic
        this->poseCallback(identity_pose);
    }

    void Controller::state_sub_callback(const std_msgs::msg::Int32 &msg)
    {
        this->state = static_cast<planner::State>(msg.data);
        if (this->state == planner::State::CENTERLINETRACK)
        {
            this->max_speed = this->base_max_speed;
            this->base_lookahead_distance = this->get_parameter("base_lookahead_distance").as_double();
            this->pure_pursuit_controller_->changeParameters(
                this->wheelbase,
                this->base_lookahead_distance,
                this->min_lookahead_distance,
                this->max_speed);
        }
        else if (this->state == planner::State::LATTICE)
        {
            this->max_speed = this->min_speed;
            this->base_lookahead_distance = this->min_lookahead_distance;
            this->pure_pursuit_controller_->changeParameters(
                this->wheelbase,
                this->base_lookahead_distance,
                this->min_lookahead_distance,
                this->max_speed);
        }
        else if (this->state == planner::State::ASTAR)
        {
            this->max_speed = this->min_speed; // Slow speed for Lattice planning // Slow speed for A* planning
            this->base_lookahead_distance = this->min_lookahead_distance;
            this->pure_pursuit_controller_->changeParameters(
                this->wheelbase,
                this->base_lookahead_distance,
                this->min_lookahead_distance,
                this->max_speed);
        }
        else if (this->state == planner::State::PARKING_ASTAR)
        {
            this->max_speed = this->min_speed; // Slow speed for Lattice planning // Slow speed for Parking A* planning
            this->base_lookahead_distance = this->min_lookahead_distance;
            this->pure_pursuit_controller_->changeParameters(
                this->wheelbase,
                this->base_lookahead_distance,
                this->min_lookahead_distance,
                this->max_speed);
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown state received: %d",
                static_cast<int>(this->state));
        }
    }

    void Controller::speedCallback(const geometry_msgs::msg::TwistWithCovarianceStamped &msg)
    {
        double wheel_radius = 0.45;                                            // Wheel radius from GaeSimInterfacePlugin.hpp
        double linear_speed = msg.twist.twist.linear.x;                        // Speed in m/s
        this->current_speed = (linear_speed / (2 * M_PI * wheel_radius)) * 60; // Convert to RPM
        static double base_brake_distance = this->brake_distance;
        this->brake_distance = (this->current_speed / 200.0) * base_brake_distance; // Update brake distance based on current speed
        this->brake_distance = std::clamp(this->brake_distance, 10.0, 35.0);
        // RCLCPP_INFO(this->get_logger(), "Current speed updated from cmd_vel_cov: %.2f RPM and %.2f m/s", this->current_speed, linear_speed);
    }

    void Controller::imuCallback(const sensor_msgs::msg::Imu &msg)
    {
        // Extract yaw rate (angular velocity around z-axis) for MPC oscillation damping
        this->current_yaw_rate_ = msg.angular_velocity.z; // rad/s
    }

    void Controller::switchControllerCallback(const std_msgs::msg::Bool &msg)
    {
        if (msg.data)
        {
            this->switch_info = true;
        }
        else if (!msg.data)
        {
            this->switch_info = false;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received invalid switch controller command.");
        }
    }

    void Controller::stopCallback(const std_msgs::msg::Bool &msg)
    {
        this->stop_topic_info = msg.data;
    }

    double Controller::speedController(double v_current, double v_ref)
    {
        double speed_error = v_ref - v_current;

        double Ukp = this->Kp_speed * speed_error;
        double Uki = this->Uki_prev_speed + this->Ki_speed * speed_error * this->Ts;

        // Anti-windup clamping for integral term
        double clamp_val = 0.1 * this->max_speed;
        Uki = std::clamp(Uki, -clamp_val, clamp_val);

        double Ukd = this->Kd_speed * (speed_error - this->error_prev_speed) / this->Ts;
        double pid_output = Ukp + Uki + Ukd;

        // Update states
        this->Uki_prev_speed = Uki;
        this->Ukd_prev_speed = Ukd;
        this->error_prev_speed = speed_error;

        // RCLCPP_INFO(this->get_logger(), "PID output: %.2f, v_ref: %.2f, v_current: %.2f", pid_output, v_ref, v_current);

        if (pid_output > 0)
        {
            pid_output = std::clamp(pid_output, this->min_speed, this->max_speed);
        }
        else
        {
            pid_output = std::clamp(pid_output, -this->max_speed, -this->min_speed);
        }

        return pid_output;
    }

    double Controller::computeDynamicDerivative(const nav_msgs::msg::Path &path_transformed)
    {
        if (path_transformed.poses.size() < 2)
            return 0.0;

        double sum_dy = 0.0;
        size_t count = 0;

        for (size_t i = 1; i < path_transformed.poses.size(); ++i)
        {
            double dy = std::abs(path_transformed.poses[i].pose.position.y -
                                 path_transformed.poses[i - 1].pose.position.y);
            double dx = std::abs(path_transformed.poses[i].pose.position.x -
                                 path_transformed.poses[i - 1].pose.position.x);
            double local_slope = dx > 0.001 ? dy / dx : 0.0;

            sum_dy += local_slope * local_slope;
            count++;
        }

        sum_dy = std::sqrt(sum_dy);

        double mean_dy = (count > 0) ? (sum_dy / count) : 0.0;

        double Ukd = this->Kd * mean_dy;

        return Ukd;
    }

    void Controller::vref_generator()
    {
        nav_msgs::msg::Path path_transformed;
        path_transformed.header.frame_id = "base_link"; // Transform to vehicle frame
        path_transformed.header.stamp = this->get_clock()->now();

        for (const auto &pose_stamped : this->path.poses)
        {
            geometry_msgs::msg::PointStamped path_point;
            path_point.header = pose_stamped.header;
            path_point.point.x = pose_stamped.pose.position.x;
            path_point.point.y = pose_stamped.pose.position.y;
            path_point.point.z = pose_stamped.pose.position.z;

            // Check if path is already in base_link
            if (this->path.header.frame_id == "base_link")
            {
                geometry_msgs::msg::PoseStamped transformed_pose = pose_stamped;
                transformed_pose.header.frame_id = "base_link";
                path_transformed.poses.push_back(transformed_pose);
            }
            else
            {
                geometry_msgs::msg::PointStamped transformed_point;
                try
                {
                    transformed_point = tf_buffer_->transform(path_point, "base_link");

                    geometry_msgs::msg::PoseStamped transformed_pose;
                    transformed_pose.header = transformed_point.header;
                    transformed_pose.pose.position.x = transformed_point.point.x;
                    transformed_pose.pose.position.y = transformed_point.point.y;
                    transformed_pose.pose.position.z = transformed_point.point.z;
                    transformed_pose.pose.orientation = pose_stamped.pose.orientation;

                    path_transformed.poses.push_back(transformed_pose);
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                    continue;
                }
            }
        }

        double error = 0.0;

        nav_msgs::msg::Path limited_path;

        for (const auto &pose_stamped : path_transformed.poses)
        {
            if (pose_stamped.pose.position.x < 0)
            {
                continue; // Ignore points behind the vehicle
            }
            limited_path.poses.push_back(pose_stamped);
            // Consider this logic to make it dynamic.
            error = error + pose_stamped.pose.position.y * pose_stamped.pose.position.y;
            if (std::sqrt(pose_stamped.pose.position.y * pose_stamped.pose.position.y + pose_stamped.pose.position.x * pose_stamped.pose.position.x) > this->brake_distance)
            {
                break; // No need to look beyond the brake distance.
            }
        }

        transformed_path_pub->publish(limited_path);

        error = std::sqrt(error);

        // Proportional term
        double Ukp = this->Kp * error;

        // Derivative term
        double Ukd = computeDynamicDerivative(path_transformed);

        // PD output
        double pd_output = Ukp + Ukd;

        // Update previous values
        this->Ukd_prev = Ukd;
        this->error_prev = error;
        {
            if (pd_output > this->max_speed)
            {
                pd_output = this->max_speed;
            }
            else if (pd_output < -this->max_speed)
            {
                pd_output = -this->max_speed;
            }
        }
        // Map PD output to speed range [min_speed, max_speed]
        double speed_range = this->max_speed - this->min_speed;
        this->referenceSpeed = this->min_speed + speed_range * std::max(0.0, std::min(1.0, (this->max_speed - pd_output) / this->max_speed));
        // RCLCPP_INFO(this->get_logger(), "PD output: %f, Reference speed: %f", pd_output, this->referenceSpeed);
    }

    void Controller::trajectoryCallback(const nav_msgs::msg::Path &path)
    {
        this->path = path;
    }

    void Controller::distanceToStopCallback(const std_msgs::msg::Float64 &msg)
    {
        this->distance_to_stop = msg.data;
        if (this->distance_to_stop < this->brake_distance)
        {
            this->apply_brake_distance_stop = true;
        }
        else
        {
            this->apply_brake_distance_stop = false;
        }
    }

    void Controller::telemetryCallback(const gae_msgs::msg::GaeTelemetry &msg)
    {
        (void)msg;
        this->last_speed_update_time = this->clock_->now(); // Update the last speed update time
    }

    nav_msgs::msg::Path Controller::extractRelevantPath(
        const geometry_msgs::msg::Pose &current_pose,
        const nav_msgs::msg::Path &full_path,
        double lookahead_distance)
    {
        nav_msgs::msg::Path relevant_path;
        relevant_path.header = full_path.header;

        if (full_path.poses.empty())
        {
            return relevant_path;
        }

        // Step 1: Find closest point on path
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;

        for (size_t i = 0; i < full_path.poses.size(); ++i)
        {
            double dx = full_path.poses[i].pose.position.x - current_pose.position.x;
            double dy = full_path.poses[i].pose.position.y - current_pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // Step 2: Extract path segment using cumulative arc length
        double cumulative_dist = 0.0;
        size_t end_idx = closest_idx;

        for (size_t i = closest_idx; i < full_path.poses.size() - 1; ++i)
        {
            double dx = full_path.poses[i + 1].pose.position.x - full_path.poses[i].pose.position.x;
            double dy = full_path.poses[i + 1].pose.position.y - full_path.poses[i].pose.position.y;
            double segment_length = std::hypot(dx, dy);

            cumulative_dist += segment_length;
            end_idx = i + 1;

            if (cumulative_dist >= lookahead_distance)
            {
                break;
            }
        }

        // Step 3: Copy the relevant segment
        for (size_t i = closest_idx; i <= end_idx && i < full_path.poses.size(); ++i)
        {
            relevant_path.poses.push_back(full_path.poses[i]);
        }

        return relevant_path;
    }

    void Controller::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        this->current_pose = msg.pose.pose;

        double current_velocity_ms = (this->current_speed / 60.0) * 2.0 * M_PI * WHEEL_RADIUS;
        double max_velocity_ms = (this->max_speed / 60.0) * 2.0 * M_PI * WHEEL_RADIUS;
        double min_velocity_ms = (this->min_speed / 60.0) * 2.0 * M_PI * WHEEL_RADIUS;

        if (this->path.poses.size() == 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Path is empty!");
            return;
        }

        bool pid_mode = true;
        double steering_angle = 0.0;
        double target_acceleration = 0.0;
        double mpc_direct_throttle = 0.0;
        bool use_direct_mpc_control = false;

        if (this->use_stanley_)
        {
            double yaw = tf2::getYaw(this->current_pose.orientation);

            auto result = stanley_controller_.compute(
                this->current_pose,
                yaw,
                current_velocity_ms,
                this->path);

            double steering_deg = result.steering * (180.0 / M_PI);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "Stanley: steer=%.2f deg, cte=%.3f m, eth=%.2f deg",
                                 steering_deg, result.cte, result.heading_error * (180.0 / M_PI));

            int raw_steering = static_cast<int>((steering_deg / 27.0 + 1.0) * 1800.0);
            raw_steering = std::clamp(raw_steering, 0, 3600);
            steering_angle = static_cast<double>(raw_steering);

            target_acceleration = 0.0;

            pid_mode = false;
        }
        else if (this->use_mpc_)
        {
            try
            {
                const double mpc_lookahead = 100.0;
                nav_msgs::msg::Path mpc_path = this->extractRelevantPath(
                    this->current_pose, this->path, mpc_lookahead);

                if (mpc_path.poses.size() < 5)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "MPC path too short (%zu points), falling back to Pure Pursuit",
                                         mpc_path.poses.size());
                    throw std::runtime_error("Path too short for MPC");
                }

                itusct::MPCControl mpc_result = mpc_controller_->solve(
                    this->current_pose,
                    mpc_path,
                    current_velocity_ms,
                    this->current_yaw_rate_,
                    this->max_mpc_steering_rad_,
                    max_velocity_ms,
                    min_velocity_ms);

                double steering_deg = mpc_result.steering * (180.0 / M_PI);
                target_acceleration = mpc_result.acceleration;

                auto diag = mpc_controller_->getDiagnostics();

                steering_deg = std::clamp(steering_deg, -27.0, 27.0);
                int raw_steering = static_cast<int>((steering_deg / 27.0 + 1.0) * 1800.0);
                raw_steering = std::clamp(raw_steering, 0, 3600);
                steering_angle = static_cast<double>(raw_steering);

                const auto &mpc_trajectory = mpc_controller_->getPredictedTrajectory();
                if (!mpc_trajectory.empty())
                {
                    nav_msgs::msg::Path predicted_path;
                    predicted_path.header.frame_id = "map";
                    predicted_path.header.stamp = this->get_clock()->now();

                    for (const auto &state : mpc_trajectory)
                    {
                        geometry_msgs::msg::PoseStamped pose_stamped;
                        pose_stamped.header = predicted_path.header;
                        pose_stamped.pose.position.x = state.x;
                        pose_stamped.pose.position.y = state.y;
                        pose_stamped.pose.position.z = 0.0;

                        tf2::Quaternion q;
                        q.setRPY(0, 0, state.yaw);
                        pose_stamped.pose.orientation = tf2::toMsg(q);

                        predicted_path.poses.push_back(pose_stamped);
                    }

                    this->predicted_trajectory_pub->publish(predicted_path);
                }

                double mpc_target_velocity_ms = diag.target_velocity;
                double mpc_target_rpm = (mpc_target_velocity_ms / (2.0 * M_PI * WHEEL_RADIUS)) * 60.0;
                this->referenceSpeed = std::clamp(mpc_target_rpm, this->min_speed, this->max_speed);

                if (target_acceleration < this->mpc_brake_acceleration_threshold_)
                {
                    double velocity_error_ms = mpc_target_velocity_ms - current_velocity_ms;
                    mpc_direct_throttle = velocity_error_ms * this->mpc_brake_gain_;
                    use_direct_mpc_control = true;

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "MPC Braking: v_current=%.2f m/s, v_target=%.2f m/s, acc=%.2f m/s², throttle=%.1f",
                                         current_velocity_ms, mpc_target_velocity_ms, target_acceleration, mpc_direct_throttle);
                }
                else
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "MPC Speed: current=%.1f rpm (%.2f m/s), MPC_target=%.1f rpm (%.2f m/s), acc=%.2f m/s²",
                                         this->current_speed, current_velocity_ms,
                                         this->referenceSpeed, mpc_target_velocity_ms,
                                         target_acceleration);
                }

                pid_mode = false;
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "MPC solve failed: %s. Falling back to Pure Pursuit.", e.what());

                steering_angle = pure_pursuit_controller_->computeSteeringAngle(
                    this->current_pose,
                    this->path,
                    this->referenceSpeed,
                    this->max_steering_angle, pid_mode);

                if (pid_mode)
                {
                    steering_angle = pid_controller.compute(steering_angle, this->Ts);
                }

                target_acceleration = 0.0;
            }
        }
        else
        {
            steering_angle = pure_pursuit_controller_->computeSteeringAngle(
                this->current_pose,
                this->path,
                this->referenceSpeed,
                this->max_steering_angle, pid_mode);

            if (pid_mode)
            {
                steering_angle = pid_controller.compute(steering_angle, this->Ts);
            }

            target_acceleration = 0.0;

            
        }

        // Publish Lookahead Marker for Pure Pursuit
        if (!this->use_stanley_ && (!this->use_mpc_ || pid_mode))
        {
            geometry_msgs::msg::PoseStamped lookahead_pose = pure_pursuit_controller_->getLookaheadTarget(
                this->current_pose,
                this->path,
                this->referenceSpeed);

            visualization_msgs::msg::Marker marker;
            marker.header = lookahead_pose.header; 
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "lookahead";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = lookahead_pose.pose;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1.0; 
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            this->look_ahead_marker_pub->publish(marker);
        }

        double distance_to_goal = std::hypot(
            this->path.poses.back().pose.position.x - this->current_pose.position.x,
            this->path.poses.back().pose.position.y - this->current_pose.position.y);

        if (distance_to_goal < this->brake_distance && this->apply_brake_distance_stop)
        {
            double deceleration = 200.0;
            double physics_limit = std::sqrt(2.0 * deceleration * distance_to_goal);
            double approach_speed = std::min(this->current_speed, physics_limit);
            this->referenceSpeed = approach_speed;
        }
        else if (!this->use_mpc_ || pid_mode)
        {
            this->vref_generator();
        }

        double pid_output;
        if (use_direct_mpc_control)
        {
            pid_output = mpc_direct_throttle;
        }
        else
        {
            pid_output = this->speedController(this->current_speed, this->referenceSpeed);
        }

        gae_msgs::msg::GaeControlCmd vehicle_cmd;
        this->applyThrottleLimit(pid_output, vehicle_cmd);

        vehicle_cmd.steering = steering_angle;
        vehicle_cmd.mechanical_brake = 0;
        vehicle_cmd.gear = 1;
        vehicle_cmd.mode_auto = 1;
        vehicle_cmd.signal = 0;

        if (stop_topic_info)
        {
            vehicle_cmd.throttle = 0;
            vehicle_cmd.brake = 10000;
            vehicle_cmd.mechanical_brake = 1;
        }

        this->autonomous_vehicle_cmd_pub->publish(vehicle_cmd);
    }

    void Controller::applyThrottleLimit(double pid_output, gae_msgs::msg::GaeControlCmd &vehicle_cmd)
    {
        rclcpp::Time current_time = this->get_clock()->now();

        if (this->last_control_time_.nanoseconds() == 0)
        {
            this->last_control_time_ = current_time;
            double dt = this->Ts;

            if (pid_output > 0)
            {
                this->output_throttle = std::min(pid_output, this->max_throttle_increase_rate_ * dt);
                vehicle_cmd.throttle = static_cast<int>(std::clamp(this->output_throttle, 0.0, this->max_speed));
                vehicle_cmd.brake = 0;
            }
            else
            {
                this->output_throttle = 0.0;
                vehicle_cmd.throttle = 0;

                double brake_value = std::abs(pid_output);
                brake_value = std::clamp(brake_value, 0.0, this->max_deceleration);

                if (brake_value > this->min_brake_threshold_)
                {
                    double normalized = brake_value / this->max_deceleration;
                    normalized = std::sqrt(normalized);

                    int brake_cmd = static_cast<int>(normalized * 10000.0);
                    brake_cmd = std::clamp(brake_cmd, 0, 10000);

                    vehicle_cmd.brake = brake_cmd;

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Brake: %.1f RPM (%.1f%%) -> cmd %d",
                                         brake_value, normalized * 100.0, brake_cmd);
                }
                else
                {
                    vehicle_cmd.brake = 0;
                }
            }
            return;
        }

        double dt = 0.0;
        try
        {
            dt = (current_time - this->last_control_time_).seconds();
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Time source mismatch, using nominal Ts: %s", e.what());
            dt = this->Ts;
            this->last_control_time_ = current_time;
        }

        if (dt <= 0.0 || dt > 1.0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Abnormal dt detected: %.3f, using Ts=%.3f", dt, this->Ts);
            dt = this->Ts;
        }

        this->last_control_time_ = current_time;

        double max_increase_this_step = this->max_throttle_increase_rate_ * dt;
        double max_decrease_this_step = this->max_throttle_decrease_rate_ * dt;

        if (pid_output > 0)
        {
            double requested_throttle = pid_output;
            double throttle_delta = requested_throttle - this->output_throttle;

            if (throttle_delta > max_increase_this_step)
            {
                requested_throttle = this->output_throttle + max_increase_this_step;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Throttle rate limited: %.2f -> %.2f (delta=%.2f, dt=%.3f)",
                                     pid_output, requested_throttle, throttle_delta, dt);
            }
            else if (throttle_delta < -max_decrease_this_step)
            {
                requested_throttle = this->output_throttle - max_decrease_this_step;
            }

            this->output_throttle = requested_throttle;
            vehicle_cmd.throttle = static_cast<int>(std::clamp(this->output_throttle, 0.0, this->max_speed));
            vehicle_cmd.brake = 0;
        }
        else
        {
            double brake_value = std::abs(pid_output);

            if (this->output_throttle > 0)
            {
                double new_throttle = this->output_throttle - max_decrease_this_step;
                if (new_throttle > 0)
                {
                    this->output_throttle = new_throttle;
                    vehicle_cmd.throttle = static_cast<int>(this->output_throttle);
                    vehicle_cmd.brake = 0;
                    return;
                }
            }

            this->output_throttle = 0.0;
            vehicle_cmd.throttle = 0;

            brake_value = std::clamp(brake_value, 0.0, this->max_deceleration);

            if (brake_value > this->min_brake_threshold_)
            {
                int brake_cmd = static_cast<int>((brake_value / this->max_deceleration) * 10000.0);
                brake_cmd = std::clamp(brake_cmd, 0, 10000);

                vehicle_cmd.brake = brake_cmd;

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Applying proportional brake: %.2f RPM -> %d cmd (dt=%.3f)",
                                     brake_value, brake_cmd, dt);
            }
            else
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Brake command %.2f below threshold %.2f, no brake applied.",
                                     brake_value, this->min_brake_threshold_);
                vehicle_cmd.brake = 0;
            }
        }
    }

} // namespace itusct

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(itusct::Controller)
