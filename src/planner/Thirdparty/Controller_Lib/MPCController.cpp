#include "MPCController.hpp"
#include <tf2/utils.h>
#include <dlib/optimization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>
#include <iostream>
#include <numeric>
#include <algorithm>

namespace itusct
{

    MPCController::MPCController(double wheelbase, int horizon, double dt)
        : wheelbase_(wheelbase),
          horizon_(horizon),
          dt_(dt),
          w_cte_base_(50.0),
          w_epsi_base_(40.0),
          w_v_base_(15.0),
          w_delta_base_(10.0),
          w_a_base_(1.0),
          w_ddelta_base_(1500.0),
          w_da_base_(10.0),
          w_omega_base_(100.0),
          w_domega_base_(500.0),
          w_cte_(50.0),
          w_epsi_(40.0),
          w_v_(1.0),
          w_delta_(10.0),
          w_a_(1.0),
          w_ddelta_(1500.0),
          w_da_(10.0),
          w_omega_(100.0),
          w_domega_(500.0),
          prev_velocity_(0.0),
          prev_acceleration_(0.0),
          prev_steering_(0.0),
          max_steering_rate_(3.0 * M_PI / 180.0), // 3 deg/step base rate
          prev_solution_steering_(horizon, 0.0),
          prev_solution_accel_(horizon, 0.0),
          adaptive_mode_enabled_(true)
    {
        oscillation_detector_.reset();
    }

    void MPCController::setWeights(const MPCWeights &weights)
    {
        w_cte_base_ = weights.w_cte;
        w_epsi_base_ = weights.w_epsi;
        w_v_base_ = weights.w_v;
        w_delta_base_ = weights.w_delta;
        w_ddelta_base_ = weights.w_ddelta;
        w_a_base_ = weights.w_a;
        w_da_base_ = weights.w_da;
        w_omega_base_ = weights.w_omega;
        w_domega_base_ = weights.w_domega;

        // Initialize active weights
        applyAdaptedWeights();
    }

    void MPCController::resetAdaptiveState()
    {
        oscillation_detector_.reset();
        max_steering_rate_ = 3.0 * M_PI / 180.0;
        applyAdaptedWeights();

        // Clear warm start
        std::fill(prev_solution_steering_.begin(), prev_solution_steering_.end(), 0.0);
        std::fill(prev_solution_accel_.begin(), prev_solution_accel_.end(), 0.0);
    }

    double MPCController::normalizeAngle(double angle) const
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // ==================== OSCILLATION DETECTION ====================

    void MPCController::updateOscillationDetector(double steering, double cte, double heading_error, double timestamp)
    {
        auto &det = oscillation_detector_;

        // Add to history
        det.steering_history.push_back(steering);
        det.cte_history.push_back(cte);
        det.heading_error_history.push_back(heading_error);
        det.timestamp_history.push_back(timestamp);

        // Maintain history size
        while (det.steering_history.size() > det.history_size)
        {
            det.steering_history.pop_front();
            det.cte_history.pop_front();
            det.heading_error_history.pop_front();
            det.timestamp_history.pop_front();
        }
    }

    double MPCController::computeSignalOscillationScore(const std::deque<double> &signal, double threshold) const
    {
        if (signal.size() < 10)
            return 0.0;

        // Compute mean
        double mean = std::accumulate(signal.begin(), signal.end(), 0.0) / signal.size();

        // Compute variance and count direction changes
        double variance = 0.0;
        int direction_changes = 0;
        int prev_direction = 0;

        for (size_t i = 0; i < signal.size(); ++i)
        {
            double centered = signal[i] - mean;
            variance += centered * centered;

            if (i > 0)
            {
                double diff = signal[i] - signal[i - 1];
                int direction = (diff > 0.001) ? 1 : ((diff < -0.001) ? -1 : 0);

                if (direction != 0 && prev_direction != 0 && direction != prev_direction)
                {
                    direction_changes++;
                }
                if (direction != 0)
                    prev_direction = direction;
            }
        }

        variance /= signal.size();
        double std_dev = std::sqrt(variance);

        // Oscillation score based on:
        // 1. Amplitude (std deviation) relative to threshold
        // 2. Frequency (direction changes)

        double amplitude_score = std::min(1.0, std_dev / (threshold * 2.0));

        // Expected direction changes for oscillation: roughly signal.size() / 2 for high freq
        double expected_changes = signal.size() / 4.0; // Moderate oscillation
        double frequency_score = std::min(1.0, direction_changes / expected_changes);

        // Combined score (both amplitude AND frequency must be present)
        return amplitude_score * frequency_score;
    }

    int MPCController::countZeroCrossings(const std::deque<double> &signal) const
    {
        if (signal.size() < 2)
            return 0;

        // Compute mean for centering
        double mean = std::accumulate(signal.begin(), signal.end(), 0.0) / signal.size();

        int crossings = 0;
        bool prev_positive = (signal[0] - mean) >= 0;

        for (size_t i = 1; i < signal.size(); ++i)
        {
            bool curr_positive = (signal[i] - mean) >= 0;
            if (curr_positive != prev_positive)
            {
                crossings++;
                prev_positive = curr_positive;
            }
        }

        return crossings;
    }

    double MPCController::estimateOscillationFrequency(const std::deque<double> &signal, double dt) const
    {
        int zero_crossings = countZeroCrossings(signal);
        if (zero_crossings < 2)
            return 0.0;

        // Frequency = (zero_crossings / 2) / total_time
        double total_time = signal.size() * dt;
        return (zero_crossings / 2.0) / total_time;
    }

    double MPCController::computeSignalAmplitude(const std::deque<double> &signal) const
    {
        if (signal.empty())
            return 0.0;

        double min_val = *std::min_element(signal.begin(), signal.end());
        double max_val = *std::max_element(signal.begin(), signal.end());

        return (max_val - min_val) / 2.0;
    }

    void MPCController::analyzeOscillation()
    {
        auto &det = oscillation_detector_;

        if (det.steering_history.size() < 20)
        {
            det.oscillation_detected = false;
            return;
        }

        // Compute oscillation scores for each signal
        det.steering_oscillation_score = computeSignalOscillationScore(
            det.steering_history, det.steering_oscillation_threshold);

        det.cte_oscillation_score = computeSignalOscillationScore(
            det.cte_history, det.cte_oscillation_threshold);

        det.heading_oscillation_score = computeSignalOscillationScore(
            det.heading_error_history, det.heading_oscillation_threshold);

        // Combined score: steering oscillation is primary indicator
        // CTE and heading oscillation confirm the effect
        det.combined_oscillation_score =
            0.5 * det.steering_oscillation_score +
            0.25 * det.cte_oscillation_score +
            0.25 * det.heading_oscillation_score;

        // Detect oscillation characteristics
        det.detected_frequency = estimateOscillationFrequency(det.steering_history, dt_);
        det.detected_amplitude = computeSignalAmplitude(det.steering_history);
        det.zero_crossing_count = countZeroCrossings(det.steering_history);

        // Determine if oscillation is occurring
        bool currently_oscillating = det.combined_oscillation_score > 0.3;

        if (currently_oscillating)
        {
            det.consecutive_oscillation_frames++;
            det.frames_since_last_oscillation = 0;
        }
        else
        {
            det.frames_since_last_oscillation++;
            if (det.frames_since_last_oscillation > 20)
            { // ~1 second without oscillation
                det.consecutive_oscillation_frames = 0;
            }
        }

        // Oscillation is "detected" if we've seen it consistently
        det.oscillation_detected = det.consecutive_oscillation_frames > 10;

        // Debug output
        if (det.combined_oscillation_score > 0.2)
        {
            // std::cout << "[OSC] score=" << det.combined_oscillation_score
            //           << " steer=" << det.steering_oscillation_score
            //           << " cte=" << det.cte_oscillation_score
            //           << " freq=" << det.detected_frequency << "Hz"
            //           << " amp=" << det.detected_amplitude * 57.3 << "deg"
            //           << " detected=" << det.oscillation_detected << std::endl;
        }
    }

    void MPCController::adaptParameters()
    {
        if (!adaptive_mode_enabled_)
        {
            oscillation_detector_.smoothness_multiplier = 1.0;
            oscillation_detector_.tracking_multiplier = 1.0;
            oscillation_detector_.steering_penalty_multiplier = 1.0;
            oscillation_detector_.rate_limit_multiplier = 1.0;
            return;
        }

        auto &det = oscillation_detector_;

        // Adaptation strategy based on oscillation severity
        double osc_score = det.combined_oscillation_score;

        if (det.oscillation_detected)
        {
            // VERY AGGRESSIVE ADAPTATION when oscillation is confirmed

            // 1. MASSIVELY increase smoothness penalty
            // This is the key to stopping oscillation
            det.smoothness_multiplier = 1.0 + osc_score * 7.0; // Up to 8x (was 5x)

            // 2. STRONGLY reduce tracking aggression
            // Stop chasing the path aggressively - smoothness is priority
            det.tracking_multiplier = std::max(0.15, 1.0 - osc_score * 0.85); // Down to 0.15x (was 0.3x)

            // 3. HEAVILY penalize large steering angles
            det.steering_penalty_multiplier = 1.0 + osc_score * 4.0; // Up to 5x (was 3x)

            // 4. ALWAYS reduce rate limit when oscillating (not just at high freq)
            // The amplitude (14-16 deg) is the problem, not just frequency
            double amp_degrees = det.detected_amplitude * 57.3;
            if (amp_degrees > 5.0)
            { // If amplitude > 5 degrees, reduce rate
                det.rate_limit_multiplier = std::max(0.3, 1.0 - (amp_degrees - 5.0) * 0.05);
            }
            else
            {
                det.rate_limit_multiplier = 0.7; // Always reduce when oscillating
            }

            // std::cout << "[ADAPT] OSCILLATION! smooth=" << det.smoothness_multiplier
            //           << " track=" << det.tracking_multiplier
            //           << " steer_pen=" << det.steering_penalty_multiplier
            //           << " rate=" << det.rate_limit_multiplier
            //           << " amp=" << amp_degrees << "deg" << std::endl;
        }
        else if (det.frames_since_last_oscillation > 40)
        { // ~2 seconds stable
            // GRADUAL RECOVERY when stable
            // Slowly return to base parameters

            double recovery_rate = 0.02; // 2% per frame toward normal

            det.smoothness_multiplier = det.smoothness_multiplier * (1.0 - recovery_rate) + 1.0 * recovery_rate;
            det.tracking_multiplier = det.tracking_multiplier * (1.0 - recovery_rate) + 1.0 * recovery_rate;
            det.steering_penalty_multiplier = det.steering_penalty_multiplier * (1.0 - recovery_rate) + 1.0 * recovery_rate;
            det.rate_limit_multiplier = det.rate_limit_multiplier * (1.0 - recovery_rate) + 1.0 * recovery_rate;

            // Clamp to reasonable bounds
            det.smoothness_multiplier = std::clamp(det.smoothness_multiplier, 1.0, 5.0);
            det.tracking_multiplier = std::clamp(det.tracking_multiplier, 0.3, 1.0);
            det.steering_penalty_multiplier = std::clamp(det.steering_penalty_multiplier, 1.0, 3.0);
            det.rate_limit_multiplier = std::clamp(det.rate_limit_multiplier, 0.3, 1.0);
        }
        else
        {
            // MILD ADAPTATION based on score even if not "detected"
            // Preemptive damping when oscillation is building

            if (osc_score > 0.15)
            {
                det.smoothness_multiplier = 1.0 + osc_score * 1.5;
                det.tracking_multiplier = std::max(0.6, 1.0 - osc_score * 0.4);
            }
        }
    }

    void MPCController::applyAdaptedWeights()
    {
        auto &det = oscillation_detector_;

        // Apply multipliers to base weights
        w_cte_ = w_cte_base_ * det.tracking_multiplier;
        w_epsi_ = w_epsi_base_ * det.tracking_multiplier;
        w_v_ = w_v_base_;
        w_delta_ = w_delta_base_ * det.steering_penalty_multiplier;
        w_ddelta_ = w_ddelta_base_ * det.smoothness_multiplier;
        w_a_ = w_a_base_;
        w_da_ = w_da_base_ * det.smoothness_multiplier;
        w_omega_ = w_omega_base_ * det.smoothness_multiplier;
        w_domega_ = w_domega_base_ * det.smoothness_multiplier;

        // Apply rate limit adaptation
        max_steering_rate_ = (3.0 * M_PI / 180.0) * det.rate_limit_multiplier;
    }

    // ==================== PATH PROCESSING ====================

    double MPCController::computePathCurvature(const LocalPath &local_path) const
    {
        if (local_path.waypoints_x.size() < 3)
            return 0.0;

        double max_curvature = 0.0;
        size_t check_points = std::min(size_t(15), local_path.waypoints_x.size() - 2);

        for (size_t i = 0; i < check_points; ++i)
        {
            double yaw_diff = normalizeAngle(local_path.waypoints_yaw[i + 1] - local_path.waypoints_yaw[i]);

            double ds = local_path.waypoints_s[i + 1] - local_path.waypoints_s[i];
            if (ds > 0.01)
            {
                double curvature = std::abs(yaw_diff) / ds;
                max_curvature = std::max(max_curvature, curvature);
            }
        }

        return max_curvature;
    }

    size_t MPCController::findClosestPointOnPath(
        double px, double py, double vehicle_yaw,
        const LocalPath &local_path,
        size_t search_start,
        size_t search_window) const
    {
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t closest_idx = search_start;

        size_t search_end = std::min(search_start + search_window, local_path.waypoints_x.size());

        // İleri yön vektörü
        double fwd_x = std::cos(vehicle_yaw);
        double fwd_y = std::sin(vehicle_yaw);

        for (size_t j = search_start; j < search_end; ++j)
        {
            double dx = local_path.waypoints_x[j] - px;
            double dy = local_path.waypoints_y[j] - py;
            double dist_sq = dx * dx + dy * dy;

            // Arkadaki noktaları cezalandır
            double dot = dx * fwd_x + dy * fwd_y;
            if (dot < 0)
            {
                // Nokta arkada - mesafeyi artır (daha az tercih edilir)
                dist_sq += 10.0; // Penalty
            }

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_idx = j;
            }
        }

        return closest_idx;
    }

    MPCController::LocalPath MPCController::extractLocalPath(
        const MPCState &state,
        const nav_msgs::msg::Path &path,
        double lookahead_dist) const
    {
        LocalPath local;

        if (path.poses.empty())
        {
            local.valid = false;
            return local;
        }

        // ============ STEP 1: EN YAKIN NOKTAYI BUL (SADECE İLERİDE OLANLAR) ============
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;

        // Aracın ileri yönü
        double forward_x = std::cos(state.yaw);
        double forward_y = std::sin(state.yaw);

        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            double dx = path.poses[i].pose.position.x - state.x;
            double dy = path.poses[i].pose.position.y - state.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            // Dot product: pozitif = önümüzde
            double dot = dx * forward_x + dy * forward_y;

            // Sadece ilerideki noktaları değerlendir (en az 0.5m önde)
            if (dot > 0.5)
            {
                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest_idx = i;
                }
            }
        }

        // Eğer hiç ileri nokta bulamadıysak (örn. U-turn sonrası)
        // En yakın noktayı al ve yön kontrolü yapma
        if (min_dist == std::numeric_limits<double>::max())
        {
            min_dist = std::numeric_limits<double>::max();
            for (size_t i = 0; i < path.poses.size(); ++i)
            {
                double dx = path.poses[i].pose.position.x - state.x;
                double dy = path.poses[i].pose.position.y - state.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest_idx = i;
                }
            }
        }

        // ============ STEP 2: YÖN KONTROLÜ (OPSIYONEL) ============
        // Seçilen noktanın yönü ile araç yönü arasındaki fark
        double path_yaw = tf2::getYaw(path.poses[closest_idx].pose.orientation);
        double yaw_diff = normalizeAngle(state.yaw - path_yaw);

        // Eğer yön farkı çok büyükse (>90°), ileriye doğru daha uygun nokta ara
        if (std::abs(yaw_diff) > M_PI / 2.0 && closest_idx + 10 < path.poses.size())
        {
            for (size_t i = closest_idx; i < std::min(closest_idx + 30, path.poses.size()); ++i)
            {
                double p_yaw = tf2::getYaw(path.poses[i].pose.orientation);
                double y_diff = normalizeAngle(state.yaw - p_yaw);

                if (std::abs(y_diff) < M_PI / 3.0) // 60 derece threshold
                {
                    closest_idx = i;
                    std::cout << "[PATH] Yaw correction: new idx=" << i
                              << " yaw_diff=" << y_diff * 57.3 << "°" << std::endl;
                    break;
                }
            }
        }

        // ============ STEP 3: CTE HESAPLA (EN YAKIN NOKTAYA GÖRE) ============
        size_t cte_idx = closest_idx;
        local.path_yaw_at_start = tf2::getYaw(path.poses[cte_idx].pose.orientation);

        double dx_cte = state.x - path.poses[cte_idx].pose.position.x;
        double dy_cte = state.y - path.poses[cte_idx].pose.position.y;
        local.initial_cte = -std::sin(local.path_yaw_at_start) * dx_cte +
                            std::cos(local.path_yaw_at_start) * dy_cte;

        // ============ STEP 4: HEADING ERROR (START NOKTASINA GÖRE) ============
        double target_yaw = tf2::getYaw(path.poses[closest_idx].pose.orientation);
        local.initial_eth = normalizeAngle(state.yaw - target_yaw);

        // ============ STEP 5: WAYPOINTS ÇIKAR (LOOKAHEAD DISTANCE) ============
        local.waypoints_x.clear();
        local.waypoints_y.clear();
        local.waypoints_yaw.clear();
        local.waypoints_s.clear();

        double accumulated_s = 0.0;
        local.waypoints_x.push_back(path.poses[closest_idx].pose.position.x);
        local.waypoints_y.push_back(path.poses[closest_idx].pose.position.y);
        local.waypoints_yaw.push_back(tf2::getYaw(path.poses[closest_idx].pose.orientation));
        local.waypoints_s.push_back(0.0);

        // İleriye doğru lookahead distance kadar path topla
        for (size_t i = closest_idx; i < path.poses.size() - 1 && accumulated_s < lookahead_dist; ++i)
        {
            double seg_dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
            double seg_dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
            double seg_len = std::hypot(seg_dx, seg_dy);
            accumulated_s += seg_len;

            local.waypoints_x.push_back(path.poses[i + 1].pose.position.x);
            local.waypoints_y.push_back(path.poses[i + 1].pose.position.y);
            local.waypoints_yaw.push_back(tf2::getYaw(path.poses[i + 1].pose.orientation));
            local.waypoints_s.push_back(accumulated_s);
        }

        local.total_length = accumulated_s;
        local.valid = local.waypoints_x.size() >= 2;

        // Debug output
        if (std::abs(local.initial_cte) > 2.0 || std::abs(local.initial_eth) > 0.5)
        {
            std::cout << "[PATH] Large error: idx=" << closest_idx
                      << " cte=" << local.initial_cte << "m"
                      << " eth=" << local.initial_eth * 57.3 << "°"
                      << " waypoints=" << local.waypoints_x.size() << std::endl;
        }

        return local;
    }

    std::tuple<double, double, double> MPCController::interpolateLocalPath(
        const LocalPath &local,
        double s) const
    {
        if (!local.valid || local.waypoints_s.empty())
        {
            return {0.0, 0.0, 0.0};
        }

        s = std::clamp(s, 0.0, local.total_length);

        size_t lo = 0, hi = local.waypoints_s.size() - 1;
        while (hi - lo > 1)
        {
            size_t mid = (lo + hi) / 2;
            if (local.waypoints_s[mid] <= s)
                lo = mid;
            else
                hi = mid;
        }

        double s0 = local.waypoints_s[lo];
        double s1 = local.waypoints_s[hi];
        double t = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;

        double x = local.waypoints_x[lo] + t * (local.waypoints_x[hi] - local.waypoints_x[lo]);
        double y = local.waypoints_y[lo] + t * (local.waypoints_y[hi] - local.waypoints_y[lo]);

        double yaw0 = local.waypoints_yaw[lo];
        double yaw1 = local.waypoints_yaw[hi];
        double yaw_diff = normalizeAngle(yaw1 - yaw0);
        double yaw = yaw0 + t * yaw_diff;

        return {x, y, yaw};
    }

    // ==================== MPC OBJECTIVE ====================

    MPCController::MPCObjective::MPCObjective(
        const MPCController *controller,
        const MPCState &initial_state,
        const LocalPath &local_path,
        double target_velocity)
        : controller_(controller),
          initial_state_(initial_state),
          local_path_(local_path),
          target_velocity_(target_velocity),
          vehicle_yaw_(initial_state.yaw)
    {
    }

    double MPCController::MPCObjective::operator()(const dlib::matrix<double, 0, 1> &x) const
    {
        double cost = 0.0;
        int N = controller_->horizon_;

        double px = initial_state_.x;
        double py = initial_state_.y;
        double th = initial_state_.yaw;
        double v = initial_state_.v;

        double prev_steer = controller_->prev_steering_;
        double L = controller_->wheelbase_;
        double dt = controller_->dt_;
        double v_ref = target_velocity_;

        // ============ BALANCED VELOCITY-ADAPTIVE SCALING ============
        double v_init = initial_state_.v; // [m/s]
        double cte_scale, epsi_scale, ddelta_scale, delta_scale;
        const char *speed_mode;

        if (v_init < 3.0)
        {
            // ================= LOW SPEED =================
            cte_scale = 5.0;
            epsi_scale = 4.0;
            ddelta_scale = 0.8;
            delta_scale = 0.8;
            speed_mode = "LOW";
        }
        else if (v_init < 7.0)
        {
            // ================= MEDIUM SPEED =================
            double t = (v_init - 3.0) / 4.0;

            cte_scale = 2.5 - 2.0 * t;
            epsi_scale = 2.0 - 1.5 * t;
            ddelta_scale = 0.8 + 2.2 * t;
            delta_scale = 0.8 + 0.7 * t;
            speed_mode = "MEDIUM";
        }
        else
        {
            // ================= HIGH SPEED =================
            double t = std::min(1.0, (v_init - 7.0) / 5.0);

            cte_scale = 0.5 - 0.3 * t;
            epsi_scale = 0.5 - 0.3 * t;

            ddelta_scale = 3.0 + 3.0 * t;
            delta_scale = 1.5 + 1.0 * t;
            speed_mode = "HIGH";
        }

        double w_cte_scaled = controller_->w_cte_ * cte_scale;
        double w_epsi_scaled = controller_->w_epsi_ * epsi_scale;
        double w_ddelta_scaled = controller_->w_ddelta_ * ddelta_scale;
        double w_delta_scaled = controller_->w_delta_ * delta_scale;

        // ============ CRITICAL FIX: STRICTLY FORWARD-ONLY WAYPOINT MATCHING ============
        size_t current_idx = 0;

        // Initialize yaw rate from IMU measurement
        double omega = initial_state_.omega;
        double prev_omega = omega;

        for (int i = 0; i < N; ++i)
        {
            double steer = x(i);
            double acc = x(i + N);

            // ============ SECOND-ORDER YAW DYNAMICS (OSCILLATION DAMPING) ============
            double omega_desired = (v / L) * std::tan(steer);

            double k_omega = 3.0;
            double omega_error = omega_desired - omega;
            double alpha = k_omega * omega_error;

            double max_alpha = 2.0;
            alpha = std::clamp(alpha, -max_alpha, max_alpha);

            omega += alpha * dt;

            double max_omega = 1.5;
            omega = std::clamp(omega, -max_omega, max_omega);

            // Kinematic update
            px += v * std::cos(th) * dt;
            py += v * std::sin(th) * dt;
            th += omega * dt;
            th = controller_->normalizeAngle(th);
            v += acc * dt;
            v = std::max(v, 0.1);

            // ============ FORWARD-ONLY SEARCH ============
            double min_dist_sq = std::numeric_limits<double>::max();
            size_t best_idx = current_idx;

            size_t search_end = std::min(current_idx + 30, local_path_.waypoints_x.size());

            for (size_t j = current_idx; j < search_end; ++j)
            {
                double dx = local_path_.waypoints_x[j] - px;
                double dy = local_path_.waypoints_y[j] - py;
                double dist_sq = dx * dx + dy * dy;

                if (dist_sq < min_dist_sq)
                {
                    min_dist_sq = dist_sq;
                    best_idx = j;
                }
            }

            current_idx = best_idx;

            if (current_idx >= local_path_.waypoints_x.size())
            {
                current_idx = local_path_.waypoints_x.size() - 1;
            }

            double ref_x = local_path_.waypoints_x[current_idx];
            double ref_y = local_path_.waypoints_y[current_idx];
            double ref_yaw = local_path_.waypoints_yaw[current_idx];

            // Compute errors
            double dx = px - ref_x;
            double dy = py - ref_y;
            double cte = -std::sin(ref_yaw) * dx + std::cos(ref_yaw) * dy;
            double eth = controller_->normalizeAngle(th - ref_yaw);

            // EXPONENTIAL time discount
            double decay_rate = 0.25;
            double time_discount = std::exp(-decay_rate * double(i));

            // Cost terms
            cost += time_discount * w_cte_scaled * cte * cte;
            cost += time_discount * w_epsi_scaled * eth * eth;
            cost += time_discount * controller_->w_v_ * (v - v_ref) * (v - v_ref);
            cost += time_discount * w_delta_scaled * steer * steer;
            cost += time_discount * controller_->w_a_ * acc * acc;

            // ============ YAW RATE PENALTIES (OSCILLATION DAMPING) ============
            cost += time_discount * controller_->w_omega_ * omega * omega;

            double omega_diff = omega - prev_omega;
            cost += time_discount * controller_->w_domega_ * omega_diff * omega_diff;
            prev_omega = omega;

            if (i > 0)
            {
                double steer_diff = steer - x(i - 1);
                cost += time_discount * w_ddelta_scaled * steer_diff * steer_diff;

                double acc_prev = x((i - 1) + N);
                double acc_diff = acc - acc_prev;
                cost += time_discount * controller_->w_da_ * acc_diff * acc_diff;
            }
            else
            {
                double steer_diff = steer - prev_steer;
                cost += time_discount * w_ddelta_scaled * steer_diff * steer_diff;

                double prev_acc = 0.0;
                if (!controller_->prev_solution_accel_.empty())
                {
                    prev_acc = controller_->prev_solution_accel_[0];
                }
                double acc_diff0 = acc - prev_acc;
                cost += time_discount * controller_->w_da_ * acc_diff0 * acc_diff0;
            }
        }

        return cost;
    }

    // ==================== MAIN SOLVE FUNCTION ====================

    MPCControl MPCController::solve(
        const geometry_msgs::msg::Pose &current_pose,
        const nav_msgs::msg::Path &path,
        double current_velocity, // Now in m/s instead of RPM
        double current_yaw_rate,
        double max_steering,
        double max_velocity, // Max velocity in m/s (was max_rpm)
        double min_velocity) // Min velocity in m/s (was min_rpm)
    {
        MPCState initial_state;
        initial_state.x = current_pose.position.x;
        initial_state.y = current_pose.position.y;
        initial_state.yaw = tf2::getYaw(current_pose.orientation);
        initial_state.v = current_velocity; // Direct velocity input
        initial_state.omega = current_yaw_rate;

        double base_lookahead = std::max(25.0, initial_state.v * horizon_ * dt_ * 1.2 + 20.0);
        LocalPath local_path = extractLocalPath(initial_state, path, base_lookahead);

        if (!local_path.valid)
        {
            MPCControl control;
            control.steering = 0.0;
            control.acceleration = 0.0; // Safe fallback
            return control;
        }

        double timestamp = 0.0;
        updateOscillationDetector(prev_steering_, local_path.initial_cte, local_path.initial_eth, timestamp);

        analyzeOscillation();
        adaptParameters();
        applyAdaptedWeights();

        double cte_threshold = 1.5;
        double eth_threshold = 0.5;

        bool large_error = (std::abs(local_path.initial_cte) > cte_threshold ||
                            std::abs(local_path.initial_eth) > eth_threshold);

        double current_v = initial_state.v;

        double horizon_distance = initial_state.v * horizon_ * dt_;
        double max_curvature_ahead = 0.0;

        for (size_t i = 0; i < local_path.waypoints_s.size() - 1; ++i)
        {
            if (local_path.waypoints_s[i] > horizon_distance)
                break;

            double yaw_diff = normalizeAngle(local_path.waypoints_yaw[i + 1] - local_path.waypoints_yaw[i]);
            double ds = local_path.waypoints_s[i + 1] - local_path.waypoints_s[i];

            if (ds > 0.01)
            {
                double curvature = std::abs(yaw_diff) / ds;
                max_curvature_ahead = std::max(max_curvature_ahead, curvature);
            }
        }

        // ============ ADAPTIVE VELOCITY PLANNING ============
        double target_velocity = max_velocity;

        // 1. CTE-based velocity reduction
        double cte_abs = std::abs(local_path.initial_cte);
        if (cte_abs > 2.0)
        {
            target_velocity = std::min(target_velocity, max_velocity * 0.4);
        }
        else if (cte_abs > 1.0)
        {
            double cte_factor = 1.0 - (cte_abs - 1.0) * 0.4;
            target_velocity = std::min(target_velocity, max_velocity * cte_factor);
        }

        // 2. Curvature-based velocity reduction
        if (max_curvature_ahead > 0.15)
        {
            double curv_factor = std::max(0.3, 1.0 - max_curvature_ahead * 4.0);
            target_velocity = std::min(target_velocity, max_velocity * curv_factor);
        }
        else if (max_curvature_ahead > 0.05)
        {
            double curv_factor = std::max(0.5, 1.0 - max_curvature_ahead * 2.0);
            target_velocity = std::min(target_velocity, max_velocity * curv_factor);
        }
        else if (max_curvature_ahead > 0.02)
        {
            target_velocity = std::min(target_velocity, max_velocity * 0.8);
        }

        // 3. Oscillation-based velocity reduction
        if (oscillation_detector_.oscillation_detected)
        {
            target_velocity = std::min(target_velocity, max_velocity * 0.5);
        }

        target_velocity = std::clamp(target_velocity, min_velocity, max_velocity);

        // ============ OPTIMIZATION SETUP ============
        dlib::matrix<double, 0, 1> x(2 * horizon_);

        bool use_warm_start = !prev_solution_steering_.empty() &&
                              !large_error &&
                              max_curvature_ahead < 0.05 &&
                              !oscillation_detector_.oscillation_detected;

        if (use_warm_start)
        {
            for (int i = 0; i < horizon_ - 1; ++i)
            {
                x(i) = prev_solution_steering_[i + 1];
                x(i + horizon_) = prev_solution_accel_[i + 1];
            }
            x(horizon_ - 1) = prev_solution_steering_.back();
            x(2 * horizon_ - 1) = 0.0;
        }
        else
        {
            double k_cte = oscillation_detector_.oscillation_detected ? 0.08 : 0.15;
            double k_eth = oscillation_detector_.oscillation_detected ? 0.2 : 0.4;

            if (large_error && !oscillation_detector_.oscillation_detected)
            {
                k_cte = 0.25;
                k_eth = 0.6;
            }

            double initial_steer = -k_cte * local_path.initial_cte - k_eth * local_path.initial_eth;
            initial_steer = std::clamp(initial_steer, -max_steering * 0.8, max_steering * 0.8);

            for (int i = 0; i < horizon_; ++i)
            {
                double decay = std::exp(-0.15 * i);
                x(i) = initial_steer * decay;
                x(i + horizon_) = 0.0;
            }
        }

        // ============ CONSTRAINTS ============
        dlib::matrix<double, 0, 1> lb(2 * horizon_), ub(2 * horizon_);

        // Acceleration limits based on vehicle dynamics
        // Typical passenger car: -8 to +3 m/s^2 (braking/acceleration)
        // Conservative for autonomous: -3 to +2.5 m/s^2
        double max_acc = large_error ? 1.0 : (max_curvature_ahead > 0.05 ? 1.5 : 2.5);
        double min_acc = large_error ? -1.5 : (max_curvature_ahead > 0.05 ? -2.0 : -3.0);

        double effective_max_steering = max_steering;
        if (oscillation_detector_.oscillation_detected)
        {
            effective_max_steering = max_steering * 0.7;
        }

        for (int i = 0; i < horizon_; ++i)
        {
            lb(i) = -effective_max_steering;
            ub(i) = effective_max_steering;
            lb(i + horizon_) = min_acc; // Negative for braking
            ub(i + horizon_) = max_acc; // Positive for acceleration
        }

        MPCObjective objective(this, initial_state, local_path, target_velocity);

        bool optimization_converged = true;
        int actual_iterations = 0;

        try
        {
            int max_iter = large_error ? 50 : (max_curvature_ahead > 0.05 ? 40 : 30);
            actual_iterations = max_iter;

            dlib::find_min_box_constrained(
                dlib::bfgs_search_strategy(),
                dlib::objective_delta_stop_strategy(1e-4, max_iter),
                objective,
                dlib::derivative(objective),
                x,
                lb,
                ub);
        }
        catch (const std::exception &e)
        {
            std::cout << "[MPC] Optimization failed: " << e.what() << std::endl;
            optimization_converged = false;

            // Fallback control
            double k_cte = large_error ? 0.2 : 0.08;
            double k_eth = large_error ? 0.5 : 0.25;
            double fallback_steer = -k_cte * local_path.initial_cte - k_eth * local_path.initial_eth;
            fallback_steer = std::clamp(fallback_steer, -max_steering, max_steering);

            double steer_diff = fallback_steer - prev_steering_;
            if (std::abs(steer_diff) > max_steering_rate_)
            {
                fallback_steer = prev_steering_ + std::copysign(max_steering_rate_, steer_diff);
            }

            MPCControl control;
            control.steering = fallback_steer;

            // Safe deceleration on failure
            double velocity_error = current_v - target_velocity;
            control.acceleration = -0.5 * velocity_error; // Proportional braking
            control.acceleration = std::clamp(control.acceleration, -1.0, 0.5);

            prev_steering_ = fallback_steer;
            prev_acceleration_ = control.acceleration;
            return control;
        }

        // ============ SAVE WARM START ============
        for (int i = 0; i < horizon_; ++i)
        {
            prev_solution_steering_[i] = x(i);
            prev_solution_accel_[i] = x(i + horizon_);
        }

        // ============ EXTRACT CONTROL ============
        double raw_opt_steer = x(0);
        double opt_steer = std::clamp(raw_opt_steer, -max_steering, max_steering);

        // Apply steering rate limit
        double steer_diff = opt_steer - prev_steering_;
        if (std::abs(steer_diff) > max_steering_rate_)
        {
            opt_steer = prev_steering_ + std::copysign(max_steering_rate_, steer_diff);
        }

        // Extract acceleration (weighted average of near-term horizon)
        double weighted_accel = 0.0;
        double weight_sum = 0.0;
        int accel_horizon = std::min(5, horizon_);

        for (int i = 0; i < accel_horizon; ++i)
        {
            double weight = std::exp(-0.2 * i); // Exponential weighting
            weighted_accel += x(i + horizon_) * weight;
            weight_sum += weight;
        }
        weighted_accel /= weight_sum;

        // ============ BUILD PREDICTED TRAJECTORY ============
        predicted_trajectory_.clear();
        MPCState state = initial_state;
        double omega = state.omega;

        for (int i = 0; i < horizon_; ++i)
        {
            double steer = x(i);
            double acc = x(i + horizon_);

            // Second-order yaw dynamics
            double omega_desired = (state.v / wheelbase_) * std::tan(steer);
            double k_omega = 3.0;
            double alpha = k_omega * (omega_desired - omega);

            double max_alpha = 2.0;
            alpha = std::clamp(alpha, -max_alpha, max_alpha);

            omega += alpha * dt_;

            double max_omega = 1.5;
            omega = std::clamp(omega, -max_omega, max_omega);

            // Kinematic update
            state.x += state.v * std::cos(state.yaw) * dt_;
            state.y += state.v * std::sin(state.yaw) * dt_;
            state.yaw += omega * dt_;
            state.yaw = normalizeAngle(state.yaw);

            state.v = std::max(0.1, state.v + acc * dt_);
            state.v = std::clamp(state.v, min_velocity, max_velocity); // Enforce velocity limits
            state.omega = omega;

            predicted_trajectory_.push_back(state);
        }

        // ============ FINALIZE CONTROL OUTPUT ============
        MPCControl control;
        control.steering = opt_steer;
        control.acceleration = weighted_accel;

        // Clamp acceleration to safe limits
        control.acceleration = std::clamp(control.acceleration, min_acc, max_acc);

        prev_steering_ = opt_steer;
        prev_acceleration_ = control.acceleration;

        // ============ DIAGNOSTICS ============
        last_diagnostics_.cte = local_path.initial_cte;
        last_diagnostics_.heading_error = local_path.initial_eth;
        last_diagnostics_.velocity = initial_state.v;
        last_diagnostics_.path_curvature = max_curvature_ahead;
        last_diagnostics_.target_velocity = target_velocity;
        last_diagnostics_.actual_acceleration = control.acceleration;
        last_diagnostics_.actual_steering_cmd = opt_steer;
        last_diagnostics_.raw_optimal_steering = raw_opt_steer;
        last_diagnostics_.oscillation_detected = oscillation_detector_.oscillation_detected;
        last_diagnostics_.oscillation_score = oscillation_detector_.combined_oscillation_score;
        last_diagnostics_.smoothness_multiplier = oscillation_detector_.smoothness_multiplier;
        last_diagnostics_.tracking_multiplier = oscillation_detector_.tracking_multiplier;
        last_diagnostics_.optimization_iterations = actual_iterations;
        last_diagnostics_.optimization_converged = optimization_converged;

        return control;
    }

} // namespace itusct