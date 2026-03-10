#ifndef GAE_SIM_INTERFACE_PLUGIN_HPP
#define GAE_SIM_INTERFACE_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gae_msgs/msg/gae_control_cmd.hpp>
#include <gae_msgs/msg/gae_telemetry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::placeholders;

namespace gazebo {

// Kinematics parameters
#define GAE_VEHICLE_STEERING_RATIO      27.3  // Ratio between steering wheel angle and tire angle
#define GAE_VEHICLE_LOCK_TO_LOCK_REVS   1.8   // Number of steering wheel turns to go from lock to lock
#define GAE_VEHICLE_MAX_STEER_ANGLE     (M_PI * GAE_VEHICLE_LOCK_TO_LOCK_REVS / GAE_VEHICLE_STEERING_RATIO)
#define GAE_VEHICLE_WHEELBASE           1.65  // Distance between front and rear axles
#define GAE_VEHICLE_TRACK_WIDTH         1.038 // Distance between front wheels

// Drag parameters
#define ROLLING_RESISTANCE_COEFF  0.08
#define AERO_DRAG_COEFF           0.35
#define GRAVITY_ACCEL             9.81
#define VEHICLE_MASS              4000.0
#define WHEEL_RADIUS              0.10
#define MAX_BRAKE_TORQUE          8000.0

// Gear states
enum { DRIVE = 0, REVERSE = 1 };

class GaeSimInterfacePlugin : public ModelPlugin {
public:
  GaeSimInterfacePlugin();
  virtual ~GaeSimInterfacePlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void Update();
  virtual void Reset();

private:
  void steeringUpdate(double time_step);
  void driveUpdateRPM(double time_step);
  void recvControlCmd(const gae_msgs::msg::GaeControlCmd::ConstSharedPtr msg);
  void physicsUpdate();
  void setAllWheelTorque(double torque);
  void odomTimerCallback();
  void safetyTimerCallback();

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_handle_;

  rclcpp::Subscription<gae_msgs::msg::GaeControlCmd>::SharedPtr sub_control_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_with_cov_;
  rclcpp::Publisher<gae_msgs::msg::GaeTelemetry>::SharedPtr pub_telemetry_;
  
  int odom_timer_count_;
  int safety_timer_count_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  ignition::math::Pose3d world_pose_;
  event::ConnectionPtr update_connection_;
  
  /// \brief The parent World
  physics::WorldPtr world_;

  physics::JointPtr steer_fl_joint_;
  physics::JointPtr steer_fr_joint_;
  physics::JointPtr wheel_rl_joint_;
  physics::JointPtr wheel_rr_joint_;
  physics::JointPtr wheel_fl_joint_;
  physics::JointPtr wheel_fr_joint_;
  physics::LinkPtr footprint_link_;
  std::string frame_id_;

  /// \brief save last_time
  common::Time last_time;

  std::string model_name_;

  // SDF parameters
  std::string robot_name_;

  // Steering values
  double right_angle_;
  double left_angle_;
  double target_angle_;
  double current_steering_angle_;

  common::PID wheel_rl_pid_;
  common::PID wheel_rr_pid_;

  double target_speed_;
  double target_steering_angle_;

  // Brakes
  double brake_cmd_;
  common::Time brake_stamp_;

  // Throttle
  double throttle_cmd_;
  common::Time throttle_stamp_;

  // Gear
  uint8_t gear_cmd_;

};

GZ_REGISTER_MODEL_PLUGIN(GaeSimInterfacePlugin)

}

#endif // GAE_SIM_INTERFACE_PLUGIN_HPP