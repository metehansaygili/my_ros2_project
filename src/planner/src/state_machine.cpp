#include "state_machine.hpp"

namespace itusct
{
    StateMachine::StateMachine(const rclcpp::NodeOptions &options)
        : Node("state_machine_exe", options)
    {
        RCLCPP_INFO(this->get_logger(), "State Machine Initialized (1-2-3-4 Mode)");

        // ==========================================
        // PARAMETERS
        // ==========================================
        this->declare_parameter<std::string>("behavioral_topic", "/behavioral");
        this->declare_parameter<std::string>("state_topic", "/vehicle_state");
        this->declare_parameter<int>("publish_rate", 10);

        std::string behavioral_topic = this->get_parameter("behavioral_topic").as_string();
        std::string state_topic = this->get_parameter("state_topic").as_string();
        int rate = this->get_parameter("publish_rate").as_int();

        // ==========================================
        // SUBSCRIBERS
        // ==========================================

        // Input Expectation: 1=CENTERLINE, 2=LATTICE, 3=ASTAR
        this->behavioral_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            behavioral_topic, 10,
            std::bind(&StateMachine::behavioralCallback, this, std::placeholders::_1));

        this->is_park_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/is_park", 10,
            std::bind(&StateMachine::isParkCallback, this, std::placeholders::_1));

        // ==========================================
        // PUBLISHERS
        // ==========================================
        this->vehicle_state_pub_ = this->create_publisher<std_msgs::msg::Int32>(state_topic, 10);

        // ==========================================
        // MAIN LOOP
        // ==========================================
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / rate),
            std::bind(&StateMachine::updateAndPublishState, this));

        // RCLCPP_INFO(this->get_logger(), "State Machine setup complete. Publishing on '%s' at %d Hz.", state_topic.c_str(), rate);
    }

    void StateMachine::behavioralCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        this->current_behavior_id_ = msg->data;
    }

    void StateMachine::isParkCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        this->is_park_mode_ = msg->data;
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        //                      "Parking Mode is now: %s", this->is_park_mode_ ? "ENABLED" : "DISABLED");
    }

    void StateMachine::updateAndPublishState()
    {
        planner::State current_state = planner::State::CENTERLINETRACK; // Default

        // ==========================================
        // PRIORITY LOGIC
        // ==========================================

        // Priority 1: If Parking Mode is active, FORCE state 4
        if (this->is_park_mode_)
        {
            current_state = planner::State::PARKING_ASTAR;
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            //                      "Parking Mode ACTIVE: Forcing state to PARKING_ASTAR");
        }
        // Priority 2: Standard Logic (1, 2, 3)
        else
        {
            switch (this->current_behavior_id_)
            {
            case 3:
                current_state = planner::State::ASTAR;
                break;
            case 2:
                current_state = planner::State::LATTICE;
                break;
            case 1:
                current_state = planner::State::CENTERLINETRACK;
                break;
            default:
                current_state = planner::State::CENTERLINETRACK;
                break;
            }
        }

        // ==========================================
        // PUBLISH
        // ==========================================
        std_msgs::msg::Int32 state_msg;
        state_msg.data = static_cast<int>(current_state);
        this->vehicle_state_pub_->publish(state_msg);
    }

    std::string StateMachine::stateToString(planner::State s)
    {
        switch (s)
        {
        case planner::State::CENTERLINETRACK:
            return "CENTERLINETRACK";
        case planner::State::LATTICE:
            return "LATTICE";
        case planner::State::ASTAR:
            return "ASTAR";
        case planner::State::PARKING_ASTAR:
            return "PARKING_ASTAR";
        default:
            return "UNKNOWN";
        }
    }

} // namespace itusct

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(itusct::StateMachine)