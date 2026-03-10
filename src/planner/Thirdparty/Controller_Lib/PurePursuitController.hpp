#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>
#include <rclcpp/rclcpp.hpp>

class PurePursuitController
{
public:
    PurePursuitController(double wheelbase,
                          double base_lookahead_distance,
                          double min_lookahead_distance,
                          double max_speed,
                            double sensitivity)
        : wheelbase_(wheelbase),
          base_lookahead_distance_(base_lookahead_distance),
          min_lookahead_distance_(min_lookahead_distance),
          max_speed_(max_speed),
          sensitivity_(sensitivity) {}

    /**
     * @brief Compute steering angle using Pure Pursuit algorithm
     * @param current_pose Current vehicle pose (from localization)
     * @param path Reference path to follow
     * @param reference_speed Current reference speed for adaptive lookahead
     * @param max_steering_angle Maximum steering angle in degrees
     * @return Steering angle command in the range [0, 3600]
     */
    double computeSteeringAngle(const geometry_msgs::msg::Pose &current_pose,
                                const nav_msgs::msg::Path &path,
                                double reference_speed, double max_steering_angle, bool for_pid)
    {
        if (path.poses.empty())
        {
            return 0.0;
        }

        // Get current yaw
        double current_yaw = tf2::getYaw(current_pose.orientation);

        // Find closest point on path
        size_t closest_index = findClosestPoint(current_pose, path);

        // Calculate adaptive lookahead distance based on speed
        double lookahead_distance = std::max(
            base_lookahead_distance_ * reference_speed / max_speed_,
            min_lookahead_distance_);

        // Find lookahead target point
        geometry_msgs::msg::PoseStamped lookahead_target =
            findLookaheadPoint(current_pose, path, closest_index, lookahead_distance);

        // Calculate steering angle using Pure Pursuit geometry
        double dx = lookahead_target.pose.position.x - current_pose.position.x;
        double dy = lookahead_target.pose.position.y - current_pose.position.y;

        // Angle to target point
        double alpha = std::atan2(dy, dx) - current_yaw;

        // Normalize alpha to [-pi, pi]
        alpha = normalizeAngle(alpha);

        // Pure Pursuit steering formula
        double actual_lookahead = std::hypot(dx, dy);
        double steering_angle = std::atan2(
                                    (2.0 * wheelbase_ * std::sin(alpha)),
                                    actual_lookahead) *
                                (180.0 / M_PI);

        steering_angle = steering_angle * sensitivity_;

        if (for_pid)
        {   
            // RCLCPP_INFO(rclcpp::get_logger("PurePursuitController"), "Steering angle for PID: %.2f", steering_angle);
            return steering_angle;
        }

        steering_angle = std::clamp(steering_angle, -max_steering_angle, max_steering_angle);

        // int raw_steering = static_cast<int>(((-steering_angle / max_steering_angle) + 1.0) * 1800.0); // Map to [0, 3600]

        // // Limit the steering command to the range [0, 3600].
        // raw_steering = std::clamp(raw_steering, 0, 3600);

        // // TODO: Mapping will be change
        // raw_steering = 3600 - raw_steering;

        return steering_angle;
    }

    /**
     * @brief Get the lookahead target point
     */
    geometry_msgs::msg::PoseStamped getLookaheadTarget(
        const geometry_msgs::msg::Pose &current_pose,
        const nav_msgs::msg::Path &path,
        double reference_speed)
    {

        if (path.poses.empty())
        {
            geometry_msgs::msg::PoseStamped empty;
            return empty;
        }

        size_t closest_index = findClosestPoint(current_pose, path);

        double lookahead_distance = std::max(
            base_lookahead_distance_ * reference_speed / max_speed_,
            min_lookahead_distance_);

        return findLookaheadPoint(current_pose, path, closest_index, lookahead_distance);
    }

    void changeParameters(double wheelbase,
                          double base_lookahead_distance,
                          double min_lookahead_distance,
                          double max_speed)
    {
        wheelbase_ = wheelbase;
        base_lookahead_distance_ = base_lookahead_distance;
        min_lookahead_distance_ = min_lookahead_distance;
        max_speed_ = max_speed;
    }

private:
    size_t findClosestPoint(const geometry_msgs::msg::Pose &current_pose,
                            const nav_msgs::msg::Path &path)
    {
        float min_dist = std::numeric_limits<float>::max();
        size_t closest_index = 0;

        for (size_t i = 0; i < path.poses.size(); i++)
        {
            float px = path.poses[i].pose.position.x;
            float py = path.poses[i].pose.position.y;
            float dist = std::hypot(
                px - current_pose.position.x,
                py - current_pose.position.y);

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_index = i;
            }
        }

        return closest_index;
    }

    geometry_msgs::msg::PoseStamped findLookaheadPoint(
        const geometry_msgs::msg::Pose &current_pose,
        const nav_msgs::msg::Path &path,
        size_t start_index,
        double lookahead_distance)
    {

        // Default to last point if no point found
        geometry_msgs::msg::PoseStamped lookahead_target = path.poses.back();

        for (size_t i = start_index; i < path.poses.size(); i++)
        {
            float px = path.poses[i].pose.position.x;
            float py = path.poses[i].pose.position.y;
            float dist = std::hypot(
                px - current_pose.position.x,
                py - current_pose.position.y);

            if (dist >= lookahead_distance)
            {
                lookahead_target = path.poses[i];
                break;
            }
            // Update to current point (handles end of path)
            lookahead_target = path.poses[i];
        }

        return lookahead_target;
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // Controller parameters
    double wheelbase_;
    double base_lookahead_distance_;
    double min_lookahead_distance_;
    double max_speed_;
    double sensitivity_;
};