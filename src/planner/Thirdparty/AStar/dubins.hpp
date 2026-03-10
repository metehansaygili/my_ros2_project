#ifndef DUBINS_PARKING_HPP
#define DUBINS_PARKING_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace DubinsParking {

    struct Pose {
        double x;
        double y;
        double theta;
        
        Pose(double _x = 0.0, double _y = 0.0, double _theta = 0.0) 
            : x(_x), y(_y), theta(_theta) {}
    };

    struct PathSegment {
        enum Type { LEFT, STRAIGHT, RIGHT };
        Type type;
        double length;
        double curvature;
    };

    struct DubinsPath {
        std::vector<Pose> poses;
        std::vector<PathSegment> segments;
        double total_length;
        double euclidean_distance;
        double path_efficiency;
        bool valid;
        bool is_simplified;
        double orientation_convergence_distance;  // NEW: Distance at which orientation is achieved
        
        DubinsPath() : total_length(0.0), euclidean_distance(0.0), 
                      path_efficiency(0.0), valid(false), is_simplified(false),
                      orientation_convergence_distance(0.0) {}
    };

    class ParkingPlanner {
    public:
        ParkingPlanner() {
            min_turning_radius_ = 4.5;
            step_size_ = 0.2;
            direct_distance_threshold_ = 1.5;      // STRICT: only <1.5m for simplified
            direct_angle_threshold_ = 0.15;        // STRICT: only <8.6° for simplified
            path_efficiency_penalty_ = 2.5;        // Increased penalty for inefficient paths
            smooth_path_bias_ = 0.9;               // STRONG bias toward LSL/RSR
            late_turn_penalty_factor_ = 0.8;       // HEAVY penalty for late turns
            early_curve_bias_ = 1.5;               // STRONG preference for early curves
            orientation_priority_weight_ = 2.0;    // NEW: Weight orientation alignment heavily
        }

        void setMinTurningRadius(double radius) {
            min_turning_radius_ = radius;
        }

        void setStepSize(double step) {
            step_size_ = step;
        }
        
        void setDirectThresholds(double distance_threshold, double angle_threshold) {
            direct_distance_threshold_ = distance_threshold;
            direct_angle_threshold_ = angle_threshold;
        }
        
        void setPathEfficiencyPenalty(double penalty) {
            path_efficiency_penalty_ = penalty;
        }
        
        void setSmoothPathBias(double bias) {
            smooth_path_bias_ = bias;
        }
        
        void setLateTurnPenaltyFactor(double factor) {
            late_turn_penalty_factor_ = factor;
        }

        void setEarlyCurveBias(double bias) {
            early_curve_bias_ = bias;
        }

        DubinsPath planPath(const Pose& start, const Pose& goal) {
            DubinsPath result;
            
            // Transform to local coordinate frame
            double dx = goal.x - start.x;
            double dy = goal.y - start.y;
            double D = std::sqrt(dx * dx + dy * dy);
            
            if (D < 1e-6) {
                result.valid = false;
                return result;
            }
            
            // Store euclidean distance
            result.euclidean_distance = D;
            
            // Calculate relative orientation difference
            double heading_to_goal = std::atan2(dy, dx);
            double start_heading_diff = normalizeAngle(start.theta - heading_to_goal);
            double goal_heading_diff = normalizeAngle(goal.theta - heading_to_goal);
            double orientation_diff = std::abs(normalizeAngle(goal.theta - start.theta));
            
            // PARKING-SPECIFIC PATH SELECTION
            // For parking, we ALMOST ALWAYS use full Dubins curves
            // This ensures proper orientation-first behavior
            
            // Only use simplified path if EXTREMELY close AND perfectly aligned already
            bool is_extremely_close = D < 1.0;  // Only within 1 meter!
            bool is_perfectly_aligned = orientation_diff < 0.1;  // ~5.7 degrees
            bool is_forward_facing = std::abs(start_heading_diff) < M_PI / 8.0;  // Within 22.5 deg
            
            // For parking, almost always use Dubins for proper orientation handling
            if (is_extremely_close && is_perfectly_aligned && is_forward_facing) {
                // Only in this very specific case (final approach) use simplified
                result = generateSimplifiedPath(start, goal, D);
                if (result.valid) {
                    return result;
                }
            }
            
            // Normalize distance by turning radius
            double d = D / min_turning_radius_;
            
            // Transform angles to local frame  
            double alpha = std::atan2(dy, dx);
            double th0 = mod2pi(start.theta - alpha);
            double th1 = mod2pi(goal.theta - alpha);
            
            // Try only 4 Dubins path types (RLR and LRL disabled for parking)
            std::vector<DubinsPath> candidates;
            std::vector<std::string> path_types = {"LSL", "RSR", "LSR", "RSL"}; // Removed "RLR", "LRL"
            std::vector<bool> is_smooth_type = {true, true, false, false}; // LSL/RSR are smoother
            
            candidates.push_back(dubinsLSL(d, th0, th1, min_turning_radius_));
            candidates.push_back(dubinsRSR(d, th0, th1, min_turning_radius_));
            candidates.push_back(dubinsLSR(d, th0, th1, min_turning_radius_));
            candidates.push_back(dubinsRSL(d, th0, th1, min_turning_radius_));
            // DISABLED: RLR and LRL paths are not suitable for parking maneuvers
            // candidates.push_back(dubinsRLR(d, th0, th1, min_turning_radius_));
            // candidates.push_back(dubinsLRL(d, th0, th1, min_turning_radius_));
            
            // Calculate path metrics with PARKING-SPECIFIC scoring
            for (size_t i = 0; i < candidates.size(); ++i) {
                auto& candidate = candidates[i];
                if (candidate.valid) {
                    candidate.euclidean_distance = D;
                    candidate.path_efficiency = candidate.total_length / D;
                    
                    // Calculate orientation convergence distance (how early does it align?)
                    candidate.orientation_convergence_distance = calculateOrientationConvergence(
                        start, goal, candidate.segments);
                    
                    // STRONG bias toward smooth paths (LSL/RSR) for parking
                    if (!is_smooth_type[i]) {
                        candidate.total_length *= (1.0 + (1.0 - smooth_path_bias_) * 1.0);  // Stronger penalty
                    }
                    
                    // HEAVY penalty for late sharp turns (parking requires early alignment!)
                    if (candidate.segments.size() >= 2) {
                        const auto& last_seg = candidate.segments.back();
                        // Check if last segment is a sharp turn
                        if (last_seg.type != PathSegment::STRAIGHT && 
                            std::abs(last_seg.curvature) > 0.12) {
                            // This is BAD for parking - we want to turn first, then go straight
                            double late_turn_penalty = last_seg.length * late_turn_penalty_factor_ * 2.0;
                            candidate.total_length += late_turn_penalty;
                        }
                        
                        // REWARD early turns followed by straight segments
                        const auto& first_seg = candidate.segments.front();
                        if (first_seg.type != PathSegment::STRAIGHT && 
                            candidate.segments.size() >= 2 &&
                            candidate.segments[1].type == PathSegment::STRAIGHT) {
                            // This is GOOD for parking - early turn then straight entry
                            double early_turn_bonus = first_seg.length * 0.3;  // 30% bonus
                            candidate.total_length -= early_turn_bonus;
                        }
                    }
                    
                    // Reward paths that achieve orientation early
                    double early_orientation_bonus = 0.0;
                    if (candidate.orientation_convergence_distance < D * 0.5) {
                        // Orientation achieved in first half of path - EXCELLENT for parking
                        early_orientation_bonus = D * 0.4;
                        candidate.total_length -= early_orientation_bonus;
                    }
                }
            }
            
            // Find best path with PARKING-OPTIMIZED selection
            double best_cost = std::numeric_limits<double>::max();
            int best_idx = -1;
            
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (candidates[i].valid) {
                    // Calculate total cost with penalties
                    double efficiency_penalty = 0.0;
                    if (candidates[i].path_efficiency > path_efficiency_penalty_) {
                        efficiency_penalty = (candidates[i].path_efficiency - path_efficiency_penalty_) * D * 0.8;
                    }
                    
                    // NEW: Add orientation priority weight
                    double orientation_penalty = 0.0;
                    if (candidates[i].orientation_convergence_distance > D * 0.6) {
                        // Orientation achieved late - bad for parking
                        orientation_penalty = D * orientation_priority_weight_ * 0.5;
                    }
                    
                    double total_cost = candidates[i].total_length + efficiency_penalty + orientation_penalty;
                    
                    if (total_cost < best_cost) {
                        best_cost = total_cost;
                        best_idx = static_cast<int>(i);
                    }
                }
            }
            
            if (best_idx >= 0) {
                result = candidates[best_idx];
                result.poses = samplePath(start, result.segments);
            }
            
            return result;
        }

    private:
        double min_turning_radius_;
        double step_size_;
        double direct_distance_threshold_;
        double direct_angle_threshold_;
        double path_efficiency_penalty_;
        double smooth_path_bias_;
        double late_turn_penalty_factor_;
        double early_curve_bias_;
        double orientation_priority_weight_;  // NEW

        double mod2pi(double angle) const {
            double v = std::fmod(angle, 2.0 * M_PI);
            if (v < 0.0) v += 2.0 * M_PI;
            return v;
        }
        
        double normalizeAngle(double angle) const {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }
        
        // Calculate at what distance the path achieves the target orientation
        double calculateOrientationConvergence(const Pose& start, const Pose& goal, 
                                               const std::vector<PathSegment>& segments) const {
            Pose current = start;
            double accumulated_distance = 0.0;
            double target_orientation = goal.theta;
            double convergence_threshold = 0.2;  // ~11.5 degrees
            
            for (const auto& segment : segments) {
                double curvature = segment.curvature;
                double length = segment.length;
                int num_steps = std::max(1, static_cast<int>(std::ceil(length / step_size_)));
                
                for (int i = 1; i <= num_steps; ++i) {
                    double step_length = (i == num_steps) ? 
                                        (length - (num_steps - 1) * step_size_) : step_size_;
                    current = integrateArc(current, curvature, step_length);
                    accumulated_distance += step_length;
                    
                    // Check if we've achieved target orientation
                    double orientation_error = std::abs(normalizeAngle(current.theta - target_orientation));
                    if (orientation_error < convergence_threshold) {
                        return accumulated_distance;
                    }
                }
            }
            
            // If never converged, return total path length
            return accumulated_distance;
        }
        
        // ORIENTATION-FIRST simplified path for parking
        // Strategy: Turn to match goal orientation FIRST, then drive straight
        DubinsPath generateSimplifiedPath(const Pose& start, const Pose& goal, double distance) const {
            DubinsPath path;
            
            // Calculate orientation difference - THIS is what we align first
            double orientation_diff = normalizeAngle(goal.theta - start.theta);
            
            // Calculate the direction from start to goal (for final positioning)
            double dx = goal.x - start.x;
            double dy = goal.y - start.y;
            double heading_to_goal = std::atan2(dy, dx);
            
            // STRATEGY: Turn to match goal orientation, then drive to position
            // This ensures we're always aligned before entering the parking spot
            
            // Phase 1: Turn to match goal orientation FIRST
            if (std::abs(orientation_diff) > 0.05) {  // ~2.9 degrees
                double turn_length = std::abs(orientation_diff) * min_turning_radius_ * early_curve_bias_;
                PathSegment::Type turn_type = (orientation_diff > 0) ? PathSegment::LEFT : PathSegment::RIGHT;
                double curvature = (orientation_diff > 0) ? (1.0 / min_turning_radius_) : (-1.0 / min_turning_radius_);
                path.segments.push_back({turn_type, turn_length, curvature});
            }
            
            // After turning to goal orientation, calculate how to reach goal position
            // We're now at start position but facing goal.theta direction
            Pose intermediate;
            intermediate.x = start.x;
            intermediate.y = start.y;
            intermediate.theta = goal.theta;
            
            // Vector from intermediate pose to goal in world frame
            double remaining_dx = goal.x - intermediate.x;
            double remaining_dy = goal.y - intermediate.y;
            
            // Project this onto the direction we're now facing
            double forward_distance = remaining_dx * std::cos(intermediate.theta) + 
                                     remaining_dy * std::sin(intermediate.theta);
            double lateral_distance = -remaining_dx * std::sin(intermediate.theta) + 
                                      remaining_dy * std::cos(intermediate.theta);
            
            // Phase 2: Small lateral correction if needed
            if (std::abs(lateral_distance) > 0.1) {
                // Need slight curve to correct lateral offset
                double correction_radius = (forward_distance * forward_distance + lateral_distance * lateral_distance) / 
                                          (2.0 * std::abs(lateral_distance));
                correction_radius = std::max(correction_radius, min_turning_radius_);
                
                double correction_angle = std::atan2(lateral_distance, forward_distance);
                double arc_length = std::abs(correction_angle) * correction_radius;
                
                PathSegment::Type correction_type = (lateral_distance > 0) ? PathSegment::LEFT : PathSegment::RIGHT;
                double correction_curvature = (lateral_distance > 0) ? (1.0 / correction_radius) : (-1.0 / correction_radius);
                
                path.segments.push_back({correction_type, arc_length, correction_curvature});
            }
            
            // Phase 3: Final straight approach (now aligned!)
            if (forward_distance > 0.2) {
                path.segments.push_back({PathSegment::STRAIGHT, forward_distance, 0.0});
            }
            
            // Calculate total length
            path.total_length = 0.0;
            for (const auto& seg : path.segments) {
                path.total_length += seg.length;
            }
            
            path.euclidean_distance = distance;
            path.path_efficiency = path.total_length / distance;
            path.valid = true;
            path.is_simplified = true;
            
            // Sample the path
            path.poses = samplePath(start, path.segments);
            
            return path;
        }

        // LSL: Left-Straight-Left
        DubinsPath dubinsLSL(double d, double alpha, double beta, double radius) const {
            DubinsPath path;
            
            double sa = std::sin(alpha);
            double sb = std::sin(beta);
            double ca = std::cos(alpha);
            double cb = std::cos(beta);
            double c_ab = std::cos(alpha - beta);
            
            double tmp = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sa - sb);
            
            if (tmp < 0.0) {
                return path;
            }
            
            double p = std::sqrt(tmp);
            double theta = std::atan2(cb - ca, d + sa - sb);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(beta - theta);
            
            // VALIDATION: Reject if first or last turn exceeds 180 degrees (π radians)
            // This prevents the car from making a full circle turn at start or end
            if (t > M_PI || q > M_PI) {
                path.valid = false;
                return path;
            }
            
            path.segments.push_back({PathSegment::LEFT, t * radius, 1.0 / radius});
            path.segments.push_back({PathSegment::STRAIGHT, p * radius, 0.0});
            path.segments.push_back({PathSegment::LEFT, q * radius, 1.0 / radius});
            path.total_length = (t + p + q) * radius;
            path.valid = true;
            
            return path;
        }

        // RSR: Right-Straight-Right
        DubinsPath dubinsRSR(double d, double alpha, double beta, double radius) const {
            DubinsPath path;
            
            double sa = std::sin(alpha);
            double sb = std::sin(beta);
            double ca = std::cos(alpha);
            double cb = std::cos(beta);
            double c_ab = std::cos(alpha - beta);
            
            double tmp = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sb - sa);
            
            if (tmp < 0.0) {
                return path;
            }
            
            double p = std::sqrt(tmp);
            double theta = std::atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(-beta + theta);
            
            // VALIDATION: Reject if first or last turn exceeds 180 degrees (π radians)
            // This prevents the car from making a full circle turn at start or end
            if (t > M_PI || q > M_PI) {
                path.valid = false;
                return path;
            }
            
            path.segments.push_back({PathSegment::RIGHT, t * radius, -1.0 / radius});
            path.segments.push_back({PathSegment::STRAIGHT, p * radius, 0.0});
            path.segments.push_back({PathSegment::RIGHT, q * radius, -1.0 / radius});
            path.total_length = (t + p + q) * radius;
            path.valid = true;
            
            return path;
        }

        // LSR: Left-Straight-Right
        DubinsPath dubinsLSR(double d, double alpha, double beta, double radius) const {
            DubinsPath path;
            
            double sa = std::sin(alpha);
            double sb = std::sin(beta);
            double ca = std::cos(alpha);
            double cb = std::cos(beta);
            double c_ab = std::cos(alpha - beta);
            
            double tmp = -2.0 + d * d + 2.0 * c_ab + 2.0 * d * (sa + sb);
            
            if (tmp < 0.0) {
                return path;
            }
            
            double p = std::sqrt(tmp);
            double theta = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(-beta + theta);
            
            // VALIDATION: Reject if first or last turn exceeds 180 degrees (π radians)
            // This prevents the car from making a full circle turn at start or end
            if (t > M_PI || q > M_PI) {
                path.valid = false;
                return path;
            }
            
            path.segments.push_back({PathSegment::LEFT, t * radius, 1.0 / radius});
            path.segments.push_back({PathSegment::STRAIGHT, p * radius, 0.0});
            path.segments.push_back({PathSegment::RIGHT, q * radius, -1.0 / radius});
            path.total_length = (t + p + q) * radius;
            path.valid = true;
            
            return path;
        }

        // RSL: Right-Straight-Left
        DubinsPath dubinsRSL(double d, double alpha, double beta, double radius) const {
            DubinsPath path;
            
            double sa = std::sin(alpha);
            double sb = std::sin(beta);
            double ca = std::cos(alpha);
            double cb = std::cos(beta);
            double c_ab = std::cos(alpha - beta);
            
            double tmp = d * d - 2.0 + 2.0 * c_ab - 2.0 * d * (sa + sb);
            
            if (tmp < 0.0) {
                return path;
            }
            
            double p = std::sqrt(tmp);
            double theta = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(beta - theta);
            
            // VALIDATION: Reject if first or last turn exceeds 180 degrees (π radians)
            // This prevents the car from making a full circle turn at start or end
            if (t > M_PI || q > M_PI) {
                path.valid = false;
                return path;
            }
            
            path.segments.push_back({PathSegment::RIGHT, t * radius, -1.0 / radius});
            path.segments.push_back({PathSegment::STRAIGHT, p * radius, 0.0});
            path.segments.push_back({PathSegment::LEFT, q * radius, 1.0 / radius});
            path.total_length = (t + p + q) * radius;
            path.valid = true;
            
            return path;
        }

        // RLR: Right-Left-Right
        DubinsPath dubinsRLR(double d, double alpha, double beta, double radius) const {
            DubinsPath path;
            
            double sa = std::sin(alpha);
            double sb = std::sin(beta);
            double ca = std::cos(alpha);
            double cb = std::cos(beta);
            double c_ab = std::cos(alpha - beta);
            
            double tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
            
            if (std::abs(tmp) > 1.0) {
                return path;
            }
            
            double p = mod2pi(2.0 * M_PI - std::acos(tmp));
            double theta = std::atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta + mod2pi(p / 2.0));
            double q = mod2pi(alpha - beta - t + mod2pi(p));
            
            path.segments.push_back({PathSegment::RIGHT, t * radius, -1.0 / radius});
            path.segments.push_back({PathSegment::LEFT, p * radius, 1.0 / radius});
            path.segments.push_back({PathSegment::RIGHT, q * radius, -1.0 / radius});
            path.total_length = (t + p + q) * radius;
            path.valid = true;
            
            return path;
        }

        // LRL: Left-Right-Left
        DubinsPath dubinsLRL(double d, double alpha, double beta, double radius) const {
            DubinsPath path;
            
            double sa = std::sin(alpha);
            double sb = std::sin(beta);
            double ca = std::cos(alpha);
            double cb = std::cos(beta);
            double c_ab = std::cos(alpha - beta);
            
            double tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (-sa + sb)) / 8.0;
            
            if (std::abs(tmp) > 1.0) {
                return path;
            }
            
            double p = mod2pi(2.0 * M_PI - std::acos(tmp));
            double theta = std::atan2(-ca + cb, d + sa - sb);
            double t = mod2pi(-alpha + theta + p / 2.0);
            double q = mod2pi(beta - alpha - t + mod2pi(p));
            
            path.segments.push_back({PathSegment::LEFT, t * radius, 1.0 / radius});
            path.segments.push_back({PathSegment::RIGHT, p * radius, -1.0 / radius});
            path.segments.push_back({PathSegment::LEFT, q * radius, 1.0 / radius});
            path.total_length = (t + p + q) * radius;
            path.valid = true;
            
            return path;
        }

        Pose integrateArc(const Pose& start, double curvature, double length) const {
            Pose result;
            
            if (std::abs(curvature) < 1e-9) {
                // Straight line
                result.x = start.x + length * std::cos(start.theta);
                result.y = start.y + length * std::sin(start.theta);
                result.theta = start.theta;
            } else {
                // Circular arc
                double delta_theta = length * curvature;
                result.theta = start.theta + delta_theta;
                
                double sin_start = std::sin(start.theta);
                double cos_start = std::cos(start.theta);
                double sin_end = std::sin(result.theta);
                double cos_end = std::cos(result.theta);
                
                result.x = start.x + (sin_end - sin_start) / curvature;
                result.y = start.y - (cos_end - cos_start) / curvature;
            }
            
            return result;
        }

        std::vector<Pose> samplePath(const Pose& start, const std::vector<PathSegment>& segments) const {
            std::vector<Pose> poses;
            Pose current = start;
            poses.push_back(current);
            
            for (const auto& segment : segments) {
                double curvature = segment.curvature;
                double length = segment.length;
                int num_steps = std::max(1, static_cast<int>(std::ceil(length / step_size_)));
                
                for (int i = 1; i <= num_steps; ++i) {
                    double step_length = (i == num_steps) ? 
                                        (length - (num_steps - 1) * step_size_) : step_size_;
                    current = integrateArc(current, curvature, step_length);
                    poses.push_back(current);
                }
            }
            
            return poses;
        }
    };

} // namespace DubinsParking

#endif // DUBINS_PARKING_HPP