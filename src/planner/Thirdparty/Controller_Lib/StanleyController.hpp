#ifndef STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER_HPP_

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

namespace itusct
{

/**
 * Stanley Controller - Used in DARPA Grand Challenge and racing applications
 * 
 * Fixed for:
 * - Large heading errors (>90°)
 * - Proper sign convention
 */
class StanleyController
{
public:
    struct StanleyParams
    {
        double k_cte = 2.0;        // Cross-track error gain
        double k_soft = 1.0;       // Softening constant
        double k_heading = 1.0;    // Heading error gain
        double max_steering = 0.471;  // Max steering in radians (27 degrees)
    };

    struct StanleyResult
    {
        double steering;  // in radians
        double cte;       // cross-track error
        double heading_error;  // heading error in radians
    };

    StanleyController() = default;
    
    void setParams(const StanleyParams& params) { params_ = params; }

    StanleyResult compute(
        const geometry_msgs::msg::Pose& pose,
        double yaw,
        double velocity,
        const nav_msgs::msg::Path& path)
    {
        StanleyResult result{0.0, 0.0, 0.0};
        
        if (path.poses.size() < 2)
            return result;

        // Find closest point on path
        size_t closest_idx = findClosestPoint(pose.position.x, pose.position.y, path);
        
        // Get path heading - use lookahead for stability
        double path_yaw = getPathHeading(path, closest_idx);
        
        // Heading error - vehicle yaw minus path yaw
        result.heading_error = normalizeAngle(yaw - path_yaw);
        
        // If heading error is too large (>90°), we're facing the wrong way
        // In this case, don't use CTE term - just focus on heading correction
        bool facing_wrong_way = std::abs(result.heading_error) > M_PI / 2.0;
        
        // Cross-track error (signed)
        double path_x = path.poses[closest_idx].pose.position.x;
        double path_y = path.poses[closest_idx].pose.position.y;
        double dx = pose.position.x - path_x;
        double dy = pose.position.y - path_y;
        
        // CTE is positive when vehicle is to the LEFT of path
        result.cte = -sin(path_yaw) * dx + cos(path_yaw) * dy;
        
        // Stanley control law
        double steering;
        
        if (facing_wrong_way)
        {
            // When facing wrong way, just correct heading (ignore CTE)
            steering = -params_.k_heading * result.heading_error;
        }
        else
        {
            // Normal Stanley: steering = -heading_error - atan(k * cte / v)
            // Negative signs because: positive heading error -> steer right (negative)
            double cte_term = atan2(params_.k_cte * result.cte, std::abs(velocity) + params_.k_soft);
            steering = -params_.k_heading * result.heading_error - cte_term;
        }
        
        // Clamp steering
        result.steering = std::clamp(steering, -params_.max_steering, params_.max_steering);
        
        return result;
    }

private:
    StanleyParams params_;

    size_t findClosestPoint(double x, double y, const nav_msgs::msg::Path& path) const
    {
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        
        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            double dx = path.poses[i].pose.position.x - x;
            double dy = path.poses[i].pose.position.y - y;
            double dist = dx*dx + dy*dy;
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    double getPathHeading(const nav_msgs::msg::Path& path, size_t idx) const
    {
        // Use orientation from path
        double qw = path.poses[idx].pose.orientation.w;
        double qz = path.poses[idx].pose.orientation.z;
        if (std::abs(qw) > 0.01 || std::abs(qz) > 0.01)
        {
            return 2.0 * atan2(qz, qw);
        }
        
        // Fallback: compute from points
        size_t next_idx = std::min(idx + 3, path.poses.size() - 1);  // Look ahead 3 points
        if (next_idx > idx)
        {
            double dx = path.poses[next_idx].pose.position.x - path.poses[idx].pose.position.x;
            double dy = path.poses[next_idx].pose.position.y - path.poses[idx].pose.position.y;
            return atan2(dy, dx);
        }
        
        return 0.0;
    }

    double normalizeAngle(double angle) const
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

} // namespace itusct

#endif // STANLEY_CONTROLLER_HPP_