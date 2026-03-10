#ifndef VIRDULT_PLANNER_HPP
#define VIRDULT_PLANNER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <rclcpp/rclcpp.hpp> 

namespace Lattice {

    struct Point {
        double x;
        double y;
    };

    struct Path {
        std::vector<Point> points;
        double lateral_offset; 
        double cost;
        bool blocked;
    };

    class Generator {
    public:
        Generator() {
            min_lookahead_ = 8.0;
            max_lookahead_ = 25.0;
            min_speed_ = 30.0;      
            max_speed_ = 300.0;
            lookahead_distance_ = min_lookahead_; 
            path_resolution_ = 0.5;     
            candidate_deltas_ = {-3.6, -3.1, -2.6, -2.1, -1.6, -1.1, -0.5, 0.0, 0.5, 1.1, 1.6, 2.1, 2.6, 3.1, 3.6};
            lethal_threshold_ = 40;   
            locked_delta_ = 0.0;
            frames_without_obstacle_ = 0;
            best_prev_idx = -1;
        }

        void setGridInfo(int width, int height, double resolution, 
                        double origin_x, double origin_y, 
                        const std::vector<int8_t>& grid_data) {
            grid_width_ = width;
            grid_height_ = height;
            resolution_ = resolution;
            grid_data_ = &grid_data; 
        }

        void setCenterline(const std::vector<Point>& centerline) {
            centerline_ = centerline;
        }

        void set_speed_limits(double min_speed, double max_speed) {
            min_speed_ = min_speed;
            max_speed_ = max_speed;
        }

        void setLookaheadDistances(double min_lookahead, double max_lookahead) {
            min_lookahead_ = min_lookahead;
            max_lookahead_ = max_lookahead;
        }

        std::vector<Point> computeTrajectory(double car_x, double car_y, double car_yaw, double current_speed, rclcpp::Logger logger) {
            last_candidates_.clear(); 
            std::vector<Path> candidates;

            if (centerline_.empty()) return {};

            double clamped_speed = std::clamp(current_speed, min_speed_, max_speed_);
            double ratio = (clamped_speed - min_speed_) / (max_speed_ - min_speed_);
            lookahead_distance_ = min_lookahead_ + ratio * (max_lookahead_ - min_lookahead_);

            size_t target_idx = 0;
            double min_diff = std::numeric_limits<double>::max();

            for(size_t i = 0; i < centerline_.size(); ++i) {
                double dist = std::sqrt(centerline_[i].x*centerline_[i].x + centerline_[i].y*centerline_[i].y);
                double diff = std::abs(dist - lookahead_distance_);
                
                if (centerline_[i].x > 0 && diff < min_diff) {
                    min_diff = diff;
                    target_idx = i;
                }
            }

            Point anchor = centerline_[target_idx];
            
            double road_yaw = 0.0;
            if (target_idx + 1 < centerline_.size()) {
                double dx = centerline_[target_idx+1].x - centerline_[target_idx].x;
                double dy = centerline_[target_idx+1].y - centerline_[target_idx].y;
                road_yaw = std::atan2(dy, dx);
            } else if (target_idx > 0) {
                double dx = centerline_[target_idx].x - centerline_[target_idx-1].x;
                double dy = centerline_[target_idx].y - centerline_[target_idx-1].y;
                road_yaw = std::atan2(dy, dx);
            }

            double nx = -std::sin(road_yaw);
            double ny = std::cos(road_yaw);

            for (double target_delta : candidate_deltas_) {
                Path path;
                path.lateral_offset = target_delta;
                path.blocked = false;
                path.cost = std::abs(target_delta);

                double goal_x = anchor.x + nx * target_delta;
                double goal_y = anchor.y + ny * target_delta;

                generateCubicSpline(goal_x, goal_y, road_yaw, path.points);
                
                if (analyzePath(path)) {
                    path.blocked = true;
                }
                
                candidates.push_back(path);
            }

            last_candidates_ = candidates;

            int centerline_idx = -1;
            bool centerline_clear = false;
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (std::abs(candidates[i].lateral_offset) < 0.05) {
                    centerline_idx = i;
                    centerline_clear = !candidates[i].blocked;
                    break;
                }
            }
            
            // DEBUG LOGGING
            if (centerline_idx != -1) {
                // RCLCPP_INFO(logger, "Centerline Blocked: %s", candidates[centerline_idx].blocked ? "YES" : "NO");
                if (candidates[centerline_idx].blocked) {
                    RCLCPP_WARN(logger, "Centerline IS BLOCKED! Looking for alternatives...");
                }
            }

            if (centerline_clear) {
                frames_without_obstacle_++;
            } else {
                frames_without_obstacle_ = 0;
            }

            if (centerline_clear && frames_without_obstacle_ >= 15) {
                locked_delta_ = 0.0;
                return candidates[centerline_idx].points;
            }

            int current_idx = -1;
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (std::abs(candidates[i].lateral_offset - locked_delta_) < 0.1) {
                    current_idx = i;
                    break;
                }
            }

            if (current_idx != -1 && !candidates[current_idx].blocked) {
                return candidates[current_idx].points;
            }

            int best_idx = -1;
            double best_cost = std::numeric_limits<double>::max();

            for (size_t i = 0; i < candidates.size(); ++i) {
                if (candidates[i].blocked) continue;

                double cost = candidates[i].cost;
                double offset = candidates[i].lateral_offset;

                bool same_side = (locked_delta_ * offset) > 0;
                if (same_side && std::abs(locked_delta_) > 0.1) {
                    cost *= 0.6;
                }

                double distance_penalty = std::abs(offset - locked_delta_) * 3.0;
                cost += distance_penalty;

                if (cost < best_cost) {
                    best_cost = cost;
                    best_idx = i;
                }

                if (best_prev_idx != -1 && std::abs(best_idx-best_prev_idx) > 11) {
                    best_idx = best_prev_idx;
                }

                best_prev_idx = best_idx; // consider edge cases

            }

            if (best_idx != -1) {
                locked_delta_ = candidates[best_idx].lateral_offset;
                RCLCPP_INFO(logger, "Switched to path with offset: %.2f", locked_delta_);
                return candidates[best_idx].points;
            }
            
            RCLCPP_ERROR(logger, "All lattice paths blocked!");
            return {};
        }

        const std::vector<Path>& getAllTrajectories() const {
            return last_candidates_;
        }

    private:
        int grid_width_, grid_height_;
        double resolution_;
        const std::vector<int8_t>* grid_data_ = nullptr;

        std::vector<Point> centerline_;
        std::vector<Path> last_candidates_;

        double min_lookahead_;
        double max_lookahead_;
        double min_speed_;
        double max_speed_;
        double lookahead_distance_;
        int best_prev_idx;

        double path_resolution_;
        std::vector<double> candidate_deltas_;
        int lethal_threshold_;
        
        double locked_delta_;
        int frames_without_obstacle_;

        void generateCubicSpline(double gx, double gy, double gyaw, std::vector<Point>& out_points) {
            out_points.clear();
            double dist = std::sqrt(gx*gx + gy*gy);
            double scale = dist * 1.2; 

            double mx0 = scale; 
            double my0 = 0.0;
            double mx1 = scale * std::cos(gyaw);
            double my1 = scale * std::sin(gyaw);

            int steps = static_cast<int>(dist / path_resolution_);
            if (steps < 5) steps = 5; 

            for (int i = 0; i <= steps; ++i) {
                double t = (double)i / steps;
                double t2 = t * t;
                double t3 = t2 * t;

                double h00 = 2*t3 - 3*t2 + 1;
                double h10 = t3 - 2*t2 + t;
                double h01 = -2*t3 + 3*t2;
                double h11 = t3 - t2;

                double x = h00*0.0 + h10*mx0 + h01*gx + h11*mx1;
                double y = h00*0.0 + h10*my0 + h01*gy + h11*my1;

                out_points.push_back({x, y});
            }

            double extension_length = 2.0;
            int ext_steps = static_cast<int>(extension_length / path_resolution_);
            double cos_yaw = std::cos(gyaw);
            double sin_yaw = std::sin(gyaw);

            for(int k = 1; k <= ext_steps; ++k) {
                double d = k * path_resolution_;
                double ex = gx + d * cos_yaw;
                double ey = gy + d * sin_yaw;
                out_points.push_back({ex, ey});
            }
        }

        bool analyzePath(Path& path) {
            if (!grid_data_) return true;
            int center_x = grid_width_ / 2;
            int center_y = grid_height_ / 2;

            for (const auto& pt : path.points) {
                int idx_x = center_x + static_cast<int>(pt.x / resolution_);
                int idx_y = center_y + static_cast<int>(pt.y / resolution_);

                if (idx_x < 0 || idx_x >= grid_width_ || idx_y < 0 || idx_y >= grid_height_) continue; 

                unsigned int index = idx_y * grid_width_ + idx_x;
                if (index >= grid_data_->size()) continue;

                int cell_value = static_cast<unsigned char>((*grid_data_)[index]); // Cast to unsigned to check 0-255 vs int8_t wrapping
                
                // if (cell_value > lethal_threshold_) {
                //    // DEBUG: Print collision
                //    // std::cout << "Collision at " << idx_x << "," << idx_y << " val=" << cell_value << std::endl;
                //    return true;
                // }
                if (cell_value > lethal_threshold_) return true;
            }
            return false; 
        }
    };
}
#endif