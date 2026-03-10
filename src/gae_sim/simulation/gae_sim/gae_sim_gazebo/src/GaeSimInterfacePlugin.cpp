#include <gae_sim_gazebo/GaeSimInterfacePlugin.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>

namespace gazebo
{

  GaeSimInterfacePlugin::GaeSimInterfacePlugin()
  {
    target_angle_ = 0.0;
    brake_cmd_ = 0.0;
    throttle_cmd_ = 0.0;
    gear_cmd_ = DRIVE;
    current_steering_angle_ = 0.0;
  }

  void GaeSimInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    if (!rclcpp::ok())
    {
      RCLCPP_FATAL(rclcpp::get_logger("GaeSimInterfacePlugin"), "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package");
    }

    world_ = model->GetWorld();
    RCLCPP_INFO(rclcpp::get_logger("GaeSimInterfacePlugin"), "Gaesim gazebo interface plugin is loading!");

    // Gazebo initialization
    steer_fl_joint_ = model->GetJoint("steer_fl_joint");
    steer_fr_joint_ = model->GetJoint("steer_fr_joint");
    wheel_rl_joint_ = model->GetJoint("wheel_rl_joint");
    wheel_rr_joint_ = model->GetJoint("wheel_rr_joint");
    wheel_fl_joint_ = model->GetJoint("wheel_fl_joint");
    wheel_fr_joint_ = model->GetJoint("wheel_fr_joint");
    footprint_link_ = model->GetLink("base_link");

    // Load SDF parameters
    robot_name_ = sdf->Get<std::string>("robot_name", "").first;

    model_name_ = model->GetName().substr(0, model->GetName().find("::"));
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GaeSimInterfacePlugin::Update, this));

    steer_fl_joint_->SetParam("fmax", 0, 99999.0);
    steer_fr_joint_->SetParam("fmax", 0, 99999.0);

    wheel_rl_pid_ = common::PID(1500, 0.0, 0);
    wheel_rr_pid_ = common::PID(1500, 0.0, 0);
    wheel_rl_pid_.SetCmdMax(3000);
    wheel_rr_pid_.SetCmdMax(3000);
    wheel_rl_pid_.SetCmdMin(-2000);
    wheel_rr_pid_.SetCmdMin(-2000);

    target_speed_ = 0.0;
    // ROS initialization
    node_handle_ = std::make_shared<rclcpp::Node>("gae_sim_control");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

    sub_control_cmd_ = node_handle_->create_subscription<gae_msgs::msg::GaeControlCmd>("vehicle/control", 1, std::bind(&GaeSimInterfacePlugin::recvControlCmd, this, std::placeholders::_1));
    pub_odom_ = node_handle_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    pub_twist_with_cov_ = node_handle_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/cmd_vel_cov", 1);
    pub_telemetry_ = node_handle_->create_publisher<gae_msgs::msg::GaeTelemetry>("/vehicle/telemetry", 1);

    odom_timer_count_ = 0;

    if (robot_name_.empty())
    {
      frame_id_ = footprint_link_->GetName();
    }
    else
    {
      frame_id_ = robot_name_ + "/" + footprint_link_->GetName();
    }

    executor_->add_node(node_handle_);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GaeSimInterfacePlugin::Update, this));

    RCLCPP_INFO(node_handle_->get_logger(), "gae_sim_gazebo plugin finished loading!");
  }

  void GaeSimInterfacePlugin::Update()
  {

    common::Time sim_time = world_->SimTime();
    double dt = (sim_time - last_time).Double();
    if (dt == 0.0)
      return;

    executor_->spin_some(std::chrono::milliseconds(100));
    driveUpdateRPM(dt);
    steeringUpdate(dt);
    physicsUpdate();

    if (odom_timer_count_++ >= 20)
    {
      odom_timer_count_ = 0;
      odomTimerCallback();
    }

    if (safety_timer_count_++ >= 400)
    {
      safety_timer_count_ = 0;
      safetyTimerCallback();
    }

    last_time = sim_time;
  }

  void GaeSimInterfacePlugin::steeringUpdate(double time_step)
  {
    const double max_rate = 400.0 * M_PI / 180.0 / GAE_VEHICLE_STEERING_RATIO;
    double max_inc = time_step * max_rate;

    if ((target_angle_ - current_steering_angle_) > max_inc)
    {
      current_steering_angle_ += max_inc;
    }
    else if ((target_angle_ - current_steering_angle_) < -max_inc)
    {
      current_steering_angle_ -= max_inc;
    }

    // Compute Ackermann steering angles for each wheel
    double t_alph = tan(current_steering_angle_);
    double left_steer = atan(GAE_VEHICLE_WHEELBASE * t_alph / (GAE_VEHICLE_WHEELBASE - 0.5 * GAE_VEHICLE_TRACK_WIDTH * t_alph));
    double right_steer = atan(GAE_VEHICLE_WHEELBASE * t_alph / (GAE_VEHICLE_WHEELBASE + 0.5 * GAE_VEHICLE_TRACK_WIDTH * t_alph));

    steer_fl_joint_->SetParam("vel", 0, 100.0 * (left_steer - steer_fl_joint_->Position(0)));
    steer_fr_joint_->SetParam("vel", 0, 100.0 * (right_steer - steer_fr_joint_->Position(0)));
  }

  void GaeSimInterfacePlugin::driveUpdateRPM(double time_step)
  {
    // Brakes have precedence over throttle
    if ((brake_cmd_ > 0) && ((last_time - brake_stamp_).Double() < 0.25))
    {
      double brake_torque = -brake_cmd_;

      // Apply brake force opposite to wheel rotation direction
      double fl_vel = wheel_fl_joint_->GetVelocity(0);
      double fr_vel = wheel_fr_joint_->GetVelocity(0);
      double rl_vel = wheel_rl_joint_->GetVelocity(0);
      double rr_vel = wheel_rr_joint_->GetVelocity(0);

      wheel_fl_joint_->SetForce(0, fl_vel > 0.01 ? brake_torque : (fl_vel < -0.01 ? -brake_torque : 0));
      wheel_fr_joint_->SetForce(0, fr_vel > 0.01 ? brake_torque : (fr_vel < -0.01 ? -brake_torque : 0));
      wheel_rl_joint_->SetForce(0, rl_vel > 0.01 ? brake_torque : (rl_vel < -0.01 ? -brake_torque : 0));
      wheel_rr_joint_->SetForce(0, rr_vel > 0.01 ? brake_torque : (rr_vel < -0.01 ? -brake_torque : 0));
    }
    else if (gear_cmd_ == DRIVE)
    {
      double left_error = wheel_rl_joint_->GetVelocity(0) - (throttle_cmd_ * 15);
      double right_error = wheel_rr_joint_->GetVelocity(0) - (throttle_cmd_ * 15);

      double left_torque = wheel_rl_pid_.Update(left_error, time_step) + 1500;
      double right_torque = wheel_rr_pid_.Update(right_error, time_step) + 1500;
      wheel_rl_joint_->SetForce(0, left_torque);
      wheel_rr_joint_->SetForce(0, right_torque);
    }
    else if (gear_cmd_ == REVERSE)
    {
      double reverse_cmd = -throttle_cmd_;
      double left_error = wheel_rl_joint_->GetVelocity(0) - (reverse_cmd * 15);
      double right_error = wheel_rr_joint_->GetVelocity(0) - (reverse_cmd * 15);

      double left_torque = wheel_rl_pid_.Update(left_error, time_step) - 1500;
      double right_torque = wheel_rr_pid_.Update(right_error, time_step) - 1500;
      wheel_rl_joint_->SetForce(0, left_torque);
      wheel_rr_joint_->SetForce(0, right_torque);
    }
  }

  void GaeSimInterfacePlugin::recvControlCmd(const gae_msgs::msg::GaeControlCmd::ConstSharedPtr msg)
  {
    throttle_cmd_ = msg->throttle / 100.0;
    target_angle_ = ((msg->steering / 1800.0) - 1) * GAE_VEHICLE_MAX_STEER_ANGLE;
    brake_cmd_ = msg->brake / 1000.0 * MAX_BRAKE_TORQUE;
    if (msg->gear == 0)
    {
      throttle_cmd_ = 0;
      brake_cmd_ = 0;
      target_angle_ = 0;
    }
    else if (msg->gear == 1)
    {
      gear_cmd_ = DRIVE;
    }
    else if (msg->gear == 2)
    {
      gear_cmd_ = REVERSE;
    }
    safety_timer_count_ = 0;
    throttle_stamp_ = last_time;
    brake_stamp_ = last_time;
  }

  void GaeSimInterfacePlugin::physicsUpdate()
  {
    // Apply rolling resistance and aerodynamic drag forces
    double rolling_resistance_torque = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS * GRAVITY_ACCEL;
    double velocity = (wheel_rl_joint_->GetVelocity(0) + wheel_rr_joint_->GetVelocity(0) + wheel_fl_joint_->GetVelocity(0) + wheel_fr_joint_->GetVelocity(0)) / 4.0;
    double drag_force = AERO_DRAG_COEFF * velocity * velocity;
    double drag_torque = drag_force * WHEEL_RADIUS;

    if (velocity > 0.0)
    {
      setAllWheelTorque(-rolling_resistance_torque);
      setAllWheelTorque(-drag_torque);
    }
    else
    {
      setAllWheelTorque(rolling_resistance_torque);
      setAllWheelTorque(drag_torque);
    }
  }

  void GaeSimInterfacePlugin::setAllWheelTorque(double torque)
  {
    wheel_rl_joint_->SetForce(0, 0.25 * torque);
    wheel_rr_joint_->SetForce(0, 0.25 * torque);
    wheel_fl_joint_->SetForce(0, 0.25 * torque);
    wheel_fr_joint_->SetForce(0, 0.25 * torque);
  }

  void GaeSimInterfacePlugin::odomTimerCallback()
  {
    world_pose_ = footprint_link_->WorldPose();
    auto current_ros_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(last_time);
    geometry_msgs::msg::Pose position;

    position.position.x = world_pose_.Pos().X();
    position.position.y = world_pose_.Pos().Y();
    position.position.z = world_pose_.Pos().Z();
    position.orientation.w = world_pose_.Rot().W();
    position.orientation.x = world_pose_.Rot().X();
    position.orientation.y = world_pose_.Rot().Y();
    position.orientation.z = world_pose_.Rot().Z();

    float yaw = world_pose_.Rot().Yaw();
    auto linear = footprint_link_->WorldLinearVel();
    auto angular = footprint_link_->WorldAngularVel();

    // Publish Twist with Covariance
    geometry_msgs::msg::TwistWithCovarianceStamped twist_cov_msg;
    twist_cov_msg.header.stamp = current_ros_time;
    twist_cov_msg.header.frame_id = "base_link";
    twist_cov_msg.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    twist_cov_msg.twist.covariance = {0.1, 0, 0, 0, 0, 0,
                                      0, 0.1, 0, 0, 0, 0,
                                      0, 0, 0.1, 0, 0, 0,
                                      0, 0, 0, 0.1, 0, 0,
                                      0, 0, 0, 0, 0.1, 0,
                                      0, 0, 0, 0, 0, 0.1};
    pub_twist_with_cov_->publish(twist_cov_msg);

    // telemetry message
    gae_msgs::msg::GaeTelemetry telemetry_msg;
    telemetry_msg.motor_velocity = (cosf(yaw) * linear.X() + sinf(yaw) * linear.Y()) * 60 / (2 * M_PI * WHEEL_RADIUS) / 4.0;
    ;
    // Calculate the inverse of ((msg->steering / 1800.0) - 1) * GAE_VEHICLE_MAX_STEER_ANGLE
    // To get back to the original steering command value:
    // steering_cmd = ((actual_steering / GAE_VEHICLE_MAX_STEER_ANGLE) + 1) * 1800.0
    telemetry_msg.actual_steering = ((this->current_steering_angle_ / GAE_VEHICLE_MAX_STEER_ANGLE) + 1) * 1800.0;
    pub_telemetry_->publish(telemetry_msg);
  }

  void GaeSimInterfacePlugin::safetyTimerCallback()
  {
    throttle_cmd_ = 0;
    brake_cmd_ = 0;
    target_angle_ = 0;
  }
  void GaeSimInterfacePlugin::Reset() {}

  GaeSimInterfacePlugin::~GaeSimInterfacePlugin() {}

} // namespace gazebo