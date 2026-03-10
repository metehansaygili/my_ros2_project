#ifndef PLANNER__STATE_MACHINE_HPP_
#define PLANNER__STATE_MACHINE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include "planner/utils.hpp"

namespace itusct
{

    class StateMachine : public rclcpp::Node
    {
    public:
        StateMachine(const rclcpp::NodeOptions &options);

    private:
        // ==========================================
        // VARIABLES
        // ==========================================
        int current_behavior_id_ = 1; // Centerline
        bool is_park_mode_ = false;

        // ==========================================
        // ROS2 INTERFACES
        // ==========================================
        
        // Publishers
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vehicle_state_pub_;
        
        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr behavioral_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_park_sub_;

        // Timer for publishing state
        rclcpp::TimerBase::SharedPtr timer_;

        // ==========================================
        // FUNCTIONS
        // ==========================================
        void behavioralCallback(const std_msgs::msg::Int32::SharedPtr msg);
        void isParkCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void updateAndPublishState();
        std::string stateToString(planner::State s);
    };

} // namespace itusct

#endif // PLANNER__STATE_MACHINE_HPP_