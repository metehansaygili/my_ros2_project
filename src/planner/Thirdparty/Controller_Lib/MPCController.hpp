#ifndef MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER_HPP_

#include <dlib/optimization.h>
#include <vector>
#include <tuple>
#include <cmath>
#include <deque>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace itusct
{
    struct MPCState
    {
        double x;
        double y;
        double yaw;
        double v;
        double omega = 0.0; // Yaw rate (rad/s) from IMU for oscillation damping
    };

    struct MPCControl
    {
        double acceleration; // Linear acceleration in m/s^2
        double steering;     // Steering angle in radians
    };

    struct MPCWeights
    {
        double w_cte = 50.0;
        double w_epsi = 40.0;
        double w_v = 1.0;
        double w_delta = 10.0;
        double w_ddelta = 1500.0;
        double w_a = 1.0;
        double w_da = 10.0;
        double w_omega = 100.0;  // Penalize high yaw rates (oscillation damping)
        double w_domega = 500.0; // Penalize yaw acceleration (smoothness)
    };

    // Oscillation detection and adaptive tuning
    struct OscillationDetector
    {
        // Circular buffers for history
        std::deque<double> steering_history;
        std::deque<double> cte_history;
        std::deque<double> heading_error_history;
        std::deque<double> timestamp_history;

        // Detection parameters
        size_t history_size = 40;
        double steering_oscillation_threshold = 0.05;
        double cte_oscillation_threshold = 0.15;
        double heading_oscillation_threshold = 0.08;

        // Oscillation metrics
        double steering_oscillation_score = 0.0;
        double cte_oscillation_score = 0.0;
        double heading_oscillation_score = 0.0;
        double combined_oscillation_score = 0.0;

        // Detected characteristics
        double detected_frequency = 0.0;
        double detected_amplitude = 0.0;
        int zero_crossing_count = 0;

        // Adaptive state
        bool oscillation_detected = false;
        int consecutive_oscillation_frames = 0;
        int frames_since_last_oscillation = 0;

        // Adaptation multipliers
        double smoothness_multiplier = 1.0;
        double tracking_multiplier = 1.0;
        double steering_penalty_multiplier = 1.0;
        double rate_limit_multiplier = 1.0;

        void reset()
        {
            steering_history.clear();
            cte_history.clear();
            heading_error_history.clear();
            timestamp_history.clear();
            steering_oscillation_score = 0.0;
            cte_oscillation_score = 0.0;
            heading_oscillation_score = 0.0;
            combined_oscillation_score = 0.0;
            detected_frequency = 0.0;
            detected_amplitude = 0.0;
            zero_crossing_count = 0;
            oscillation_detected = false;
            consecutive_oscillation_frames = 0;
            frames_since_last_oscillation = 100;
            smoothness_multiplier = 1.0;
            tracking_multiplier = 1.0;
            steering_penalty_multiplier = 1.0;
            rate_limit_multiplier = 1.0;
        }
    };

    // Diagnostic info
    struct MPCDiagnostics
    {
        double cte;
        double heading_error;
        double velocity;
        double path_curvature;
        double target_velocity;     // Target velocity in m/s
        double actual_acceleration; // Commanded acceleration in m/s^2
        double actual_steering_cmd;
        double raw_optimal_steering;
        bool oscillation_detected;
        double oscillation_score;
        double smoothness_multiplier;
        double tracking_multiplier;
        int optimization_iterations;
        bool optimization_converged;
        std::string adaptation_status;
    };

    class MPCController
    {
    public:
        MPCController(double wheelbase, int horizon, double dt);

        void setWeights(const MPCWeights &weights);

        MPCControl solve(
            const geometry_msgs::msg::Pose &current_pose,
            const nav_msgs::msg::Path &path,
            double current_velocity, // Current velocity in m/s
            double current_yaw_rate, // From IMU angular_velocity.z
            double max_steering,     // Maximum steering angle in radians
            double max_velocity,     // Maximum velocity in m/s
            double min_velocity);    // Minimum velocity in m/s

        std::vector<MPCState> getPredictedTrajectory() const { return predicted_trajectory_; }

        MPCDiagnostics getDiagnostics() const { return last_diagnostics_; }

        const OscillationDetector &getOscillationDetector() const { return oscillation_detector_; }

        void resetAdaptiveState();

        void setAdaptiveMode(bool enable) { adaptive_mode_enabled_ = enable; }
        bool isAdaptiveModeEnabled() const { return adaptive_mode_enabled_; }

    private:
        double wheelbase_;
        int horizon_;
        double dt_;

        // Base weights
        double w_cte_base_;
        double w_epsi_base_;
        double w_v_base_;
        double w_delta_base_;
        double w_a_base_;
        double w_ddelta_base_;
        double w_da_base_;
        double w_omega_base_;
        double w_domega_base_;

        // Active weights
        double w_cte_;
        double w_epsi_;
        double w_v_;
        double w_delta_;
        double w_a_;
        double w_ddelta_;
        double w_da_;
        double w_omega_;
        double w_domega_;

        double prev_velocity_;
        double prev_acceleration_;
        mutable double prev_steering_;
        double max_steering_rate_;

        std::vector<double> prev_solution_steering_;
        std::vector<double> prev_solution_accel_;

        std::vector<MPCState> predicted_trajectory_;

        OscillationDetector oscillation_detector_;
        bool adaptive_mode_enabled_ = true;
        MPCDiagnostics last_diagnostics_;

        struct LocalPath
        {
            bool valid = false;
            double initial_cte = 0.0;
            double initial_eth = 0.0;
            double path_yaw_at_start = 0.0;
            double total_length = 0.0;
            std::vector<double> waypoints_x;
            std::vector<double> waypoints_y;
            std::vector<double> waypoints_yaw;
            std::vector<double> waypoints_s;
        };

        // Core functions
        double normalizeAngle(double angle) const;

        LocalPath extractLocalPath(
            const MPCState &state,
            const nav_msgs::msg::Path &path,
            double lookahead_dist) const;

        std::tuple<double, double, double> interpolateLocalPath(
            const LocalPath &local,
            double s) const;

        double computePathCurvature(const LocalPath &local_path) const;

        size_t findClosestPointOnPath(
            double px, double py, double vehicle_yaw,
            const LocalPath &local_path,
            size_t search_start,
            size_t search_window) const;

        // Oscillation detection
        void updateOscillationDetector(double steering, double cte, double heading_error, double timestamp);
        void analyzeOscillation();
        void adaptParameters();
        void applyAdaptedWeights();

        double computeSignalOscillationScore(const std::deque<double> &signal, double threshold) const;
        int countZeroCrossings(const std::deque<double> &signal) const;
        double estimateOscillationFrequency(const std::deque<double> &signal, double dt) const;
        double computeSignalAmplitude(const std::deque<double> &signal) const;

        class MPCObjective
        {
        public:
            MPCObjective(
                const MPCController *controller,
                const MPCState &initial_state,
                const LocalPath &local_path,
                double target_velocity);

            double operator()(const dlib::matrix<double, 0, 1> &x) const;

        private:
            const MPCController *controller_;
            MPCState initial_state_;
            LocalPath local_path_;
            double target_velocity_;
            double vehicle_yaw_;
        };
    };

} // namespace itusct

#endif // MPC_CONTROLLER_HPP_