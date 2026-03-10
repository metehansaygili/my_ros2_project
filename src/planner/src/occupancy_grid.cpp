#include "occupancy_grid.hpp"

// TODO: Implement the updation of the occupancy grid based on lanelet boundaries.
// TODO: Implement message filters to synchronize the localization and cluster messages.
// TODO: There are some bugs with the turns. I do not know it is related to lanelet. And there is a need for recovering from if the vehicle is outside of the lanes due to poor control or etc.
// TODO: Right and left lanelets should be free since lane change is ok.
namespace itusct
{
    OccupancyGrid::OccupancyGrid(const rclcpp::NodeOptions &options)
        : Node("occupancy_grid_exe", options),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "occupancy_grid is initialized!");

        this->declare_parameter<double>("grid_resolution", 0.1);
        this->declare_parameter<int>("grid_size", 100);

        this->declare_parameter<double>("origin_pose._x", 0.0);
        this->declare_parameter<double>("origin_pose._y", 0.0);
        this->declare_parameter<std::string>("osm_path", "./map.osm");

        this->declare_parameter<std::string>("pose_topic", "/localization/ndt_pose");
        this->declare_parameter<std::string>("occupancy_grid_topic", "/occupancy_grid");
        this->declare_parameter<std::string>("cluster_topic", "/all_bottom_clusters");
        this->declare_parameter<std::string>("path_topic", "/path_ids");

        this->declare_parameter<double>("vehicle_width", 1.8);  // Vehicle width (in meters)
        this->declare_parameter<double>("vehicle_length", 4.2); // Vehicle length (in meters)
        this->declare_parameter<double>("safety_margin", 0.3);  // Safety margin (in meters)
        
        // Obstacle inflation parameters
        this->declare_parameter<double>("base_obstacle_radius", 0.6);  // Base obstacle size (meters)
        this->declare_parameter<double>("obstacle_core_radius", 2.0);  // Obstacle core radius (pixels)
        this->declare_parameter<double>("lateral_clearance_factor", 1.3);  // Dynamic obstacle lateral clearance multiplier
        this->declare_parameter<double>("prediction_time_min", 0.5);  // Minimum prediction time (seconds)
        this->declare_parameter<double>("prediction_time_max", 1.0);  // Maximum prediction time (seconds)
        this->declare_parameter<double>("max_prediction_distance", 3.0);  // Maximum longitudinal prediction (meters)
        
        // Obstacle tracking parameters
        this->declare_parameter<double>("association_distance", 2.0);
        this->declare_parameter<double>("prob_hit", 0.7);
        this->declare_parameter<double>("prob_miss", 0.3);
        this->declare_parameter<double>("prob_threshold_render", 0.4);
        this->declare_parameter<double>("prob_threshold_delete", 0.1);
        this->declare_parameter<double>("alpha_position", 0.3);
        this->declare_parameter<double>("alpha_velocity", 0.5);
        this->declare_parameter<double>("static_velocity_threshold", 0.5);
        this->declare_parameter<double>("prediction_decay_rate", 0.95);

        this->declare_parameter<bool>("debug_mode", false);
        this->declare_parameter<bool>("camera_mode", false);

        this->grid_resolution_ = this->get_parameter("grid_resolution").as_double();
        this->grid_size_ = this->get_parameter("grid_size").as_int();
        this->osm_path = this->get_parameter("osm_path").as_string();

        this->origin_x = this->get_parameter("origin_pose._x").as_double();
        this->origin_y = this->get_parameter("origin_pose._y").as_double();

        this->vehicle_width = this->get_parameter("vehicle_width").as_double();
        this->vehicle_length = this->get_parameter("vehicle_length").as_double();
        this->safety_margin = this->get_parameter("safety_margin").as_double();
        
        this->base_obstacle_radius_ = this->get_parameter("base_obstacle_radius").as_double();
        this->obstacle_core_radius_ = this->get_parameter("obstacle_core_radius").as_double();
        this->lateral_clearance_factor_ = this->get_parameter("lateral_clearance_factor").as_double();
        this->prediction_time_min_ = this->get_parameter("prediction_time_min").as_double();
        this->prediction_time_max_ = this->get_parameter("prediction_time_max").as_double();
        this->max_prediction_distance_ = this->get_parameter("max_prediction_distance").as_double();
        
        this->association_distance_ = this->get_parameter("association_distance").as_double();
        this->prob_hit_ = this->get_parameter("prob_hit").as_double();
        this->prob_miss_ = this->get_parameter("prob_miss").as_double();
        this->prob_threshold_render_ = this->get_parameter("prob_threshold_render").as_double();
        this->prob_threshold_delete_ = this->get_parameter("prob_threshold_delete").as_double();
        this->alpha_position_ = this->get_parameter("alpha_position").as_double();
        this->alpha_velocity_ = this->get_parameter("alpha_velocity").as_double();
        this->static_velocity_threshold_ = this->get_parameter("static_velocity_threshold").as_double();
        this->prediction_decay_rate_ = this->get_parameter("prediction_decay_rate").as_double();

        this->debug_mode = this->get_parameter("debug_mode").as_bool();
        this->camera_mode_ = this->get_parameter("camera_mode").as_bool();

        std::string OCCUPANCY_GRID_TOPIC = this->get_parameter("occupancy_grid_topic").as_string();
        std::string CLUSTER_TOPIC = this->get_parameter("cluster_topic").as_string();
        std::string PATH_TOPIC = this->get_parameter("path_topic").as_string();
        std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Camera Mode: %s", this->camera_mode_ ? "Enabled" : "Disabled");

        this->readLaneletMap();

       // initialize subscribers and publishers
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(OCCUPANCY_GRID_TOPIC, 10);
    lanelet_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lanelet_markers", 10);
    
    // Add tracked obstacles publisher
    tracked_obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracked_obstacles", 10);

    // Initialize Sync only if NOT in camera mode (or if we still want it for other reasons)
    // But user wants to run WITHOUT localization.
    
    if (this->camera_mode_)
    {
        RCLCPP_INFO(this->get_logger(), "Camera Mode: Subscribing directly to clusters (No Sync).");
        this->cluster_sub_ = this->create_subscription<gae_msgs::msg::GaeBottomPointsArray>(
            CLUSTER_TOPIC, 10, std::bind(&OccupancyGrid::clustersCallback, this, std::placeholders::_1));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Normal Mode: Using Synchronized Subscribers.");
        this->cluster_sub_filt_.subscribe(this, CLUSTER_TOPIC);
        this->pose_sub_filt_.subscribe(this, POSE_TOPIC);

        this->sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), this->cluster_sub_filt_, this->pose_sub_filt_);
        
        this->sync_->registerCallback(
            std::bind(&OccupancyGrid::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    this->path_sub = this->create_subscription<gae_msgs::msg::GaePathIds>(
        PATH_TOPIC, 10, std::bind(&OccupancyGrid::pathCallback, this, std::placeholders::_1));

    // Inform about node initialization.
        RCLCPP_INFO(this->get_logger(),
                    "Grid_resolution: %.2f and grid_size: %d",
                    this->grid_resolution_, this->grid_size_);
    }

    void OccupancyGrid::syncedCallback(
        const gae_msgs::msg::GaeBottomPointsArray::ConstSharedPtr clusters_msg,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg)
    {
        this->pose = pose_msg->pose.pose;
        clustersCallback(*clusters_msg);
    }
    
    double OccupancyGrid::logOdds(double probability) const
    {
        // Clamp probability to avoid log(0) or log(1)
        probability = std::max(0.001, std::min(0.999, probability));
        return std::log(probability / (1.0 - probability));
    }
    
    double OccupancyGrid::probability(double log_odds) const
    {
        return 1.0 / (1.0 + std::exp(-log_odds));
    }
    
    void OccupancyGrid::updateTracks(const std::vector<ObstacleMeasurement>& measurements)
    {
        auto current_time = this->get_clock()->now();
        
        // Measurements are already in base_link frame - no transformation needed!
        
        // STEP 1: Decay probability for tracks not recently seen
        // NOTE: In base_link frame, we DON'T predict position!
        // Static obstacles will naturally move relative to base_link as vehicle moves
        // Dynamic obstacles need next observation to update position
        for (auto& track : tracked_obstacles_)
        {
            double dt = (current_time - track.last_seen).seconds();
            
            if (dt > 0.001 && dt < 2.0)
            {
                // Only decay probability - NO position prediction in base_link!
                // Position prediction in ego frame causes strange motion artifacts
                double decay_factor = std::pow(prediction_decay_rate_, dt);
                track.probability *= decay_factor;
                track.log_odds = logOdds(track.probability);
            }
        }
        
        // STEP 2: Match measurements to existing tracks
        std::vector<bool> measurement_matched(measurements.size(), false);
        std::vector<bool> track_matched(tracked_obstacles_.size(), false);
        
        for (size_t t = 0; t < tracked_obstacles_.size(); ++t)
        {
            auto& track = tracked_obstacles_[t];
            double best_dist = association_distance_;
            int best_idx = -1;
            
            for (size_t m = 0; m < measurements.size(); ++m)
            {
                if (measurement_matched[m]) continue;
                
                double dx = measurements[m].x - track.x_base;
                double dy = measurements[m].y - track.y_base;
                double dist = std::sqrt(dx*dx + dy*dy);
                
                if (dist < best_dist)
                {
                    best_dist = dist;
                    best_idx = static_cast<int>(m);
                }
            }
            
            if (best_idx >= 0)
            {
                // MATCHED: Update track with measurement
                measurement_matched[best_idx] = true;
                track_matched[t] = true;
                
                const auto& meas = measurements[best_idx];
                double dt = (current_time - track.last_seen).seconds();
                
                // Update velocity estimate
                if (dt > 0.05 && dt < 1.0)
                {
                    double new_vx = (meas.x - track.x_base) / dt;
                    double new_vy = (meas.y - track.y_base) / dt;
                    track.vx = alpha_velocity_ * new_vx + (1.0 - alpha_velocity_) * track.vx;
                    track.vy = alpha_velocity_ * new_vy + (1.0 - alpha_velocity_) * track.vy;
                    
                    // Classify as static or dynamic
                    double speed = std::sqrt(track.vx * track.vx + track.vy * track.vy);
                    track.is_static = (speed < static_velocity_threshold_);
                }
                
                // Smooth position (higher weight for measurement)
                track.x_base = alpha_position_ * meas.x + (1.0 - alpha_position_) * track.x_base;
                track.y_base = alpha_position_ * meas.y + (1.0 - alpha_position_) * track.y_base;
                
                // Bayesian update: INCREASE probability (we saw it!)
                double log_odds_hit = std::log(prob_hit_ / (1.0 - prob_hit_));
                track.log_odds += log_odds_hit;
                track.probability = probability(track.log_odds);
                track.probability = std::min(0.99, track.probability); // Cap at 0.99
                
                track.last_seen = current_time;
            }
            else
            {
                // NOT MATCHED: Bayesian update: DECREASE probability (we didn't see it)
                double log_odds_miss = std::log(prob_miss_ / (1.0 - prob_miss_));
                track.log_odds += log_odds_miss;
                track.probability = probability(track.log_odds);
            }
        }
        
        // STEP 3: Create new tracks for unmatched measurements
        for (size_t m = 0; m < measurements.size(); ++m)
        {
            if (!measurement_matched[m])
            {
                TrackedObstacle new_track;
                new_track.x_base = measurements[m].x;
                new_track.y_base = measurements[m].y;
                new_track.vx = 0.0;
                new_track.vy = 0.0;
                new_track.probability = 0.5; // Initial uncertainty
                new_track.log_odds = logOdds(0.5);
                new_track.last_seen = current_time;
                new_track.created_at = current_time;
                new_track.id = next_obstacle_id_++;
                new_track.is_static = true; // Assume static until proven dynamic
                tracked_obstacles_.push_back(new_track);
            }
        }
        
        // STEP 4: Remove tracks with very low probability
        tracked_obstacles_.erase(
            std::remove_if(tracked_obstacles_.begin(), tracked_obstacles_.end(),
                [this](const TrackedObstacle& t) { 
                    return t.probability < prob_threshold_delete_; 
                }),
            tracked_obstacles_.end()
        );
        
        // Log tracking statistics
        if (debug_mode)
        {
            int high_prob = 0;
            for (const auto& t : tracked_obstacles_)
            {
                if (t.probability >= prob_threshold_render_) high_prob++;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Tracked obstacles: %zu (rendering: %d)", tracked_obstacles_.size(), high_prob);
        }
    }

    void OccupancyGrid::clustersCallback(const gae_msgs::msg::GaeBottomPointsArray &msg)
    {
        if (this->camera_mode_)
        {
             // Simulate a pose for camera mode (Identity Pose)
             // We set the class member 'pose' directly, which is what the logic below uses.
             this->pose.position.x = 0.0;
             this->pose.position.y = 0.0;
             this->pose.position.z = 0.0;
             this->pose.orientation.w = 1.0;
             this->pose.orientation.x = 0.0;
             this->pose.orientation.y = 0.0;
             this->pose.orientation.z = 0.0;
             
             // Fall through to the processing logic below
        }
        current_measurements_.clear();
        auto stamp = rclcpp::Time(msg.header.stamp);

        for (const auto &cluster : msg.clusters)
        {
            // Calculate cluster centroid
            double sum_x = 0.0;
            double sum_y = 0.0;
            int num_points = static_cast<int>(cluster.bottom_points_array.size());

            for (const auto &point : cluster.bottom_points_array)
            {
                sum_x += point.x;
                sum_y += point.y;
            }

            if (num_points > 0)
            {
                ObstacleMeasurement meas;
                meas.x = sum_x / num_points;
                meas.y = sum_y / num_points;
                meas.stamp = stamp;
                current_measurements_.push_back(meas);
            }
        }

        updateTracks(current_measurements_);
        
        // Debug: Log tracking info
        if (debug_mode && !tracked_obstacles_.empty())
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Measurements: %zu, Tracked: %zu, First track: (%.2f, %.2f) prob=%.2f",
                current_measurements_.size(), tracked_obstacles_.size(),
                tracked_obstacles_[0].x_base, tracked_obstacles_[0].y_base,
                tracked_obstacles_[0].probability);
        }
        
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        occupancy_grid.header.stamp = this->get_clock()->now();
        
        if (this->camera_mode_)
        {
            occupancy_grid.header.frame_id = "base_link";
        }
        else
        {
            occupancy_grid.header.frame_id = "map";
        }
        
        occupancy_grid.info.resolution = this->grid_resolution_;
        occupancy_grid.info.width = this->grid_size_;
        occupancy_grid.info.height = this->grid_size_;

        int start_x = this->grid_size_ / 2;
        int start_y = this->grid_size_ / 2;
        
        double offset = (this->grid_size_ * this->grid_resolution_) / 2.0;

        if (this->camera_mode_)
        {
            // In base_link, vehicle is at (0,0), yaw is 0
            // Grid centered on vehicle
            occupancy_grid.info.origin.position.x = -offset;
            occupancy_grid.info.origin.position.y = -offset;
            occupancy_grid.info.origin.orientation.w = 1.0;
            occupancy_grid.info.origin.orientation.x = 0.0;
            occupancy_grid.info.origin.orientation.y = 0.0;
            occupancy_grid.info.origin.orientation.z = 0.0;
        }
        else
        {
            double yaw = tf2::getYaw(this->pose.orientation);
            double dx = -offset * std::cos(yaw) + offset * std::sin(yaw);
            double dy = -offset * std::sin(yaw) - offset * std::cos(yaw);

            occupancy_grid.info.origin.position.x = this->pose.position.x + dx;
            occupancy_grid.info.origin.position.y = this->pose.position.y + dy;
            occupancy_grid.info.origin.orientation = this->pose.orientation;
        }

        std::vector<int8_t> grid_data(this->grid_size_ * this->grid_size_, 100);

        // ----- LANELET PATH MASK -----
        cv::Mat lanelet_mask = cv::Mat::ones(this->grid_size_, this->grid_size_, CV_8U) * 100;

        // SKIP LANELET MASK IN CAMERA MODE (No global map alignment)
        if (!this->camera_mode_)
        {
            if (this->lanelet_map_ == nullptr)
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Lanelet map is not loaded!");
                return;
            }
            if (this->path_.lanes.empty())
            {
                return;
            }

            // Create a marker array for lanelet visualization
            visualization_msgs::msg::MarkerArray lanelet_markers;

            visualization_msgs::msg::Marker lanelet_marker_left;
            lanelet_marker_left.header.frame_id = occupancy_grid.header.frame_id;
            lanelet_marker_left.header.stamp = this->get_clock()->now();
            lanelet_marker_left.ns = "lanelet_markers_left";
            lanelet_marker_left.id = 1;
            lanelet_marker_left.type = visualization_msgs::msg::Marker::LINE_STRIP;
            lanelet_marker_left.action = visualization_msgs::msg::Marker::ADD;
            lanelet_marker_left.scale.x = 0.1; // Line width
            lanelet_marker_left.color.r = 1.0f;
            lanelet_marker_left.color.g = 0.0f;
            lanelet_marker_left.color.b = 0.0f;
            lanelet_marker_left.color.a = 1.0f;

            visualization_msgs::msg::Marker lanelet_marker_right;
            lanelet_marker_right.header.frame_id = occupancy_grid.header.frame_id;
            lanelet_marker_right.header.stamp = this->get_clock()->now();
            lanelet_marker_right.ns = "lanelet_markers_right";
            lanelet_marker_right.id = 2;
            lanelet_marker_right.type = visualization_msgs::msg::Marker::LINE_STRIP;
            lanelet_marker_right.action = visualization_msgs::msg::Marker::ADD;
            lanelet_marker_right.scale.x = 0.1; // Line width
            lanelet_marker_right.color.r = 0.0f;
            lanelet_marker_right.color.g = 0.0f;
            lanelet_marker_right.color.b = 1.0f;
            lanelet_marker_right.color.a = 1.0f;

            std::vector<cv::Point> contour_points;
            std::vector<lanelet::BasicPoint2d> centerline_points;

            gae_msgs::msg::GaePathIds path_ids;

            for (lanelet::ConstLanelet lane : this->lanelet_map_->laneletLayer)
            {
                //   RCLCPP_INFO(this->get_logger(), "%lu. lane id: %ld", i++, lane.id());
                path_ids.lanes.push_back(lane.id());
                for (const auto &point : lane.centerline2d())
                {
                    centerline_points.push_back(lanelet::BasicPoint2d(point.x(), point.y()));
                }
            }

            double yaw = tf2::getYaw(this->pose.orientation); // Re-calculate yaw for the loop

            for (uint16_t lanelet_id : path_ids.lanes)
            {
                if (!this->lanelet_map_->laneletLayer.exists(lanelet::Id(lanelet_id)))
                    continue;
                const auto &lanelet = this->lanelet_map_->laneletLayer.get(lanelet::Id(lanelet_id));

                contour_points.clear();

                auto transform_point = [&](const lanelet::ConstPoint3d &pt) -> std::optional<cv::Point>
                {
                    double dx = pt.x() - occupancy_grid.info.origin.position.x;
                    double dy = pt.y() - occupancy_grid.info.origin.position.y;
                    double c = std::cos(-yaw);
                    double s = std::sin(-yaw);
                    double rot_x = dx * c - dy * s;
                    double rot_y = dx * s + dy * c;
                    int x = static_cast<int>(rot_x / this->grid_resolution_);
                    int y = static_cast<int>(rot_y / this->grid_resolution_);
                    return cv::Point(x, y);
                };

                // Append left bound points
                for (const auto &pt : lanelet.leftBound())
                {
                    auto opt_point = transform_point(pt);
                    if (opt_point)
                    {
                        contour_points.push_back(*opt_point);
                        if (this->debug_mode)
                        {
                            geometry_msgs::msg::PointStamped in_point;
                            in_point.header.frame_id = "map";
                            in_point.point.x = pt.x();
                            in_point.point.y = pt.y();
                            in_point.point.z = 0.0;
                            lanelet_marker_left.points.push_back(in_point.point);
                        }
                    }
                }

                // Append right bound points in reverse
                const auto &rb = lanelet.rightBound();
                for (int i = static_cast<int>(rb.size()) - 1; i >= 0; --i)
                {
                    const auto &pt = rb[i];
                    auto opt_point = transform_point(pt);
                    if (opt_point)
                    {
                        contour_points.push_back(*opt_point);

                        if (this->debug_mode)
                        {
                            geometry_msgs::msg::PointStamped in_point;
                            in_point.header.frame_id = "map";
                            in_point.point.x = pt.x();
                            in_point.point.y = pt.y();
                            in_point.point.z = 0.0;
                            lanelet_marker_right.points.push_back(in_point.point);
                        }
                    }
                }

                if (contour_points.size() < 3)
                {
                    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Too few points for polygon fill, skipping.");
                    continue;
                }

                // Convert to Clipper2 polygon
                std::vector<Clipper2Lib::PointD> subject_path;
                for (const auto &pt : contour_points)
                {
                    subject_path.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y));
                }
                // Define the grid boundary for clipping
                std::vector<Clipper2Lib::PointD> clip_rect = {
                    {0.0, 0.0},
                    {static_cast<double>(lanelet_mask.cols - 1), 0.0},
                    {static_cast<double>(lanelet_mask.cols - 1), static_cast<double>(lanelet_mask.rows - 1)},
                    {0.0, static_cast<double>(lanelet_mask.rows - 1)}
                };
            
                Clipper2Lib::ClipperD clipper;
                clipper.AddSubject({subject_path});
                clipper.AddClip({clip_rect});
                std::vector<std::vector<Clipper2Lib::PointD>> solution;
                clipper.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, solution);

                // Fill the clipped polygon
                for (const auto &poly : solution)
                {
                    std::vector<cv::Point> poly_cv;
                    for (const auto &pt : poly)
                    {
                        poly_cv.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
                    }
                    std::vector<std::vector<cv::Point>> fill_contour = {poly_cv};
                    cv::fillPoly(lanelet_mask, fill_contour, cv::Scalar(0)); // 0 = free
                }
            }

            if (this->debug_mode)
            {
                // Publish lanelet markers
                lanelet_markers.markers.push_back(lanelet_marker_left);
                lanelet_markers.markers.push_back(lanelet_marker_right);
                lanelet_marker_pub_->publish(lanelet_markers);
            }
        }
        else 
        {
            // Camera Mode: Entire grid (except obstacles) is conceptually "free" or driveable
            // for now, we just clear the lanelet mask so it doesn't block anything.
            lanelet_mask = cv::Mat::zeros(this->grid_size_, this->grid_size_, CV_8U);
        }

        // ----- DYNAMIC OBSTACLES WITH LANE-AWARE INFLATION -----
        cv::Mat obstacle_mask_float = cv::Mat::zeros(this->grid_size_, this->grid_size_, CV_32F);
        
        // Vehicle dimensions for inflation
        double vehicle_half_diag = std::sqrt(std::pow(this->vehicle_width / 2.0, 2) +
                                             std::pow(this->vehicle_length / 2.0, 2));
        
        // Render tracked obstacles with velocity-based ellipsoidal inflation
        for (const auto &track : tracked_obstacles_)
        {
            if (track.probability < prob_threshold_render_)
                continue;
            
            // Transform track position from base_link to grid coordinates
            // Grid is centered on vehicle, so base_link coordinates map directly
            // but we need to account for the grid origin offset
            int center_x = static_cast<int>((track.x_base / this->grid_resolution_)) + this->grid_size_ / 2;
            int center_y = static_cast<int>((track.y_base / this->grid_resolution_)) + this->grid_size_ / 2;
            
            if (center_x < 0 || center_x >= this->grid_size_ || center_y < 0 || center_y >= this->grid_size_)
                continue;
            
            // NOTE: Removed overly restrictive lane boundary check here
            // Obstacles at lane edges should still be rendered for safety
            
            double speed = std::sqrt(track.vx * track.vx + track.vy * track.vy);
            
            if (track.is_static || speed < this->static_velocity_threshold_)
            {
                // Static obstacle: circular inflation with lane constraints
                double total_radius_m = this->base_obstacle_radius_ + vehicle_half_diag + this->safety_margin;
                int radius_px = static_cast<int>(std::ceil(total_radius_m / this->grid_resolution_));
                
                for (int dy = -radius_px; dy <= radius_px; ++dy)
                {
                    for (int dx = -radius_px; dx <= radius_px; ++dx)
                    {
                        int px = center_x + dx;
                        int py = center_y + dy;
                        
                        if (px >= 0 && px < this->grid_size_ && py >= 0 && py < this->grid_size_)
                        {
                            double dist_sq = dx*dx + dy*dy;
                            double dist = std::sqrt(dist_sq);
                            
                            if (dist <= radius_px)
                            {
                                // Skip inflation pixels that are far from obstacle AND outside lane
                                // But always render the obstacle core for safety
                                bool is_obstacle_core = (dist <= this->obstacle_core_radius_);
                                bool is_far_from_center = (dist > radius_px * 0.7);  // Only skip outer 30%
                                bool is_outside_lane = (lanelet_mask.at<uint8_t>(py, px) != 0);
                                
                                // Only skip if far from center AND outside lane
                                if (is_outside_lane && is_far_from_center && !is_obstacle_core)
                                    continue;
                                
                                // Distance-based decay for softer edges
                                float decay = 1.0f - (dist / radius_px);
                                float prob_value = static_cast<float>(track.probability) * decay;
                                
                                float current_val = obstacle_mask_float.at<float>(py, px);
                                obstacle_mask_float.at<float>(py, px) = std::max(current_val, prob_value);
                            }
                        }
                    }
                }
            }
            else
            {
                // Dynamic obstacle: ellipsoidal inflation along velocity
                double vx_norm = track.vx / speed;
                double vy_norm = track.vy / speed;
                
                // Ellipse parameters
                double semi_minor_m = (this->base_obstacle_radius_ + vehicle_half_diag + this->safety_margin) * 
                                      this->lateral_clearance_factor_;
                
                double prediction_time = std::min(this->prediction_time_max_, 
                                                 std::max(this->prediction_time_min_, speed / 10.0));
                double longitudinal_extension_m = std::min(speed * prediction_time, this->max_prediction_distance_);
                double semi_major_m = this->base_obstacle_radius_ + vehicle_half_diag + this->safety_margin + 
                                     longitudinal_extension_m;
                
                double semi_minor_px = semi_minor_m / this->grid_resolution_;
                double semi_major_px = semi_major_m / this->grid_resolution_;
                
                int bbox_radius = static_cast<int>(std::ceil(semi_major_px)) + 2;
                
                for (int dy = -bbox_radius; dy <= bbox_radius; ++dy)
                {
                    for (int dx = -bbox_radius; dx <= bbox_radius; ++dx)
                    {
                        int px = center_x + dx;
                        int py = center_y + dy;
                        
                        if (px >= 0 && px < this->grid_size_ && py >= 0 && py < this->grid_size_)
                        {
                            // Transform to obstacle-aligned frame
                            double along = dx * vx_norm + dy * vy_norm;
                            double perp = -dx * vy_norm + dy * vx_norm;
                            
                            double ellipse_val = (along * along) / (semi_major_px * semi_major_px) +
                                                 (perp * perp) / (semi_minor_px * semi_minor_px);
                            
                            if (ellipse_val <= 1.0)
                            {
                                // Calculate distance from obstacle center
                                double dist_sq = dx*dx + dy*dy;
                                double dist = std::sqrt(dist_sq);
                                
                                // Only skip inflation pixels far from center AND outside lane
                                bool is_obstacle_core = (dist <= this->obstacle_core_radius_);
                                bool is_far_from_center = (ellipse_val > 0.5);  // Only skip outer 50% of ellipse
                                bool is_outside_lane = (lanelet_mask.at<uint8_t>(py, px) != 0);
                                
                                if (is_outside_lane && is_far_from_center && !is_obstacle_core)
                                    continue;
                                
                                double decay_factor = 1.0 - std::sqrt(ellipse_val);
                                float prob_value = static_cast<float>(track.probability * decay_factor);
                                
                                float current_val = obstacle_mask_float.at<float>(py, px);
                                obstacle_mask_float.at<float>(py, px) = std::max(current_val, prob_value);
                            }
                        }
                    }
                }
            }
        }
        
        // Convert to 8-bit for final grid
        cv::Mat obstacle_mask;
        obstacle_mask_float.convertTo(obstacle_mask, CV_8U, 255.0);
        
        // Debug: Count non-zero obstacle pixels
        if (debug_mode)
        {
            int obstacle_pixels = cv::countNonZero(obstacle_mask);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Obstacle mask: %d non-zero pixels", obstacle_pixels);
        }

        // ----- COMBINE WITH PROBABILISTIC VALUES -----
        int obstacles_rendered = 0;
        for (int y = 0; y < this->grid_size_; ++y)
        {
            for (int x = 0; x < this->grid_size_; ++x)
            {
                int idx = y * this->grid_size_ + x;

                uint8_t obstacle_value = obstacle_mask.at<uint8_t>(y, x);
                bool is_lanelet_free = (lanelet_mask.at<uint8_t>(y, x) == 0);

                if (obstacle_value > 0)
                {
                    // Render obstacles - they override everything
                    if (is_lanelet_free)
                    {
                        // Probabilistic occupancy: map [0,255] to [0,100]
                        int occupancy = static_cast<int>((obstacle_value / 255.0) * 100.0);
                        occupancy = std::min(100, std::max(0, occupancy));
                        grid_data[idx] = static_cast<int8_t>(occupancy);
                        obstacles_rendered++;
                    }
                }
                else if (is_lanelet_free)
                {
                    grid_data[idx] = 0; // Free space
                }
                else
                {
                    grid_data[idx] = 50; // Unknown (outside lane)
                }
            }
        }
        
        if (debug_mode && obstacles_rendered > 0)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Rendered %d obstacle cells in grid", obstacles_rendered);
        }

        // Start cell visualization
        grid_data[start_y * this->grid_size_ + start_x] = 31;

        occupancy_grid.data = grid_data;
        occupancy_grid_pub_->publish(occupancy_grid);
        occupancy_ready = true;
        
        // Debug: Publish tracked obstacles as markers in base_link frame
        if (debug_mode)
        {
            visualization_msgs::msg::MarkerArray marker_array;
            
            // Delete old markers
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = msg.header.frame_id; // base_link frame
            delete_marker.header.stamp = this->get_clock()->now();
            delete_marker.ns = "tracked_obstacles";
            delete_marker.id = 0;
            delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(delete_marker);
            
            // Add markers for each tracked obstacle
            int marker_id = 1;
            for (const auto& track : tracked_obstacles_)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = msg.header.frame_id; // base_link frame
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "tracked_obstacles";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                marker.pose.position.x = track.x_base;
                marker.pose.position.y = track.y_base;
                marker.pose.position.z = 0.5;
                marker.pose.orientation.w = 1.0;
                
                marker.scale.x = 1.0; // diameter
                marker.scale.y = 1.0;
                marker.scale.z = 1.0; // height
                
                // Color based on probability (red=low, green=high)
                marker.color.r = 1.0 - track.probability;
                marker.color.g = track.probability;
                marker.color.b = 0.0;
                marker.color.a = 0.7;
                
                // Add text with probability
                marker.text = "P=" + std::to_string(static_cast<int>(track.probability * 100)) + "%";
                
                marker_array.markers.push_back(marker);
            }
            
            tracked_obstacles_pub_->publish(marker_array);
        }
    }

    void OccupancyGrid::localization_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        this->pose = msg.pose.pose;
    }

    void OccupancyGrid::pathCallback(const gae_msgs::msg::GaePathIds &msg)
    {
        this->path_ = msg;
    }

    std::optional<cv::Point> OccupancyGrid::transform_point(const lanelet::ConstPoint3d &pt) const
    {
        double yaw = tf2::getYaw(this->pose.orientation);

        double dx = pt.x() - this->pose.position.x;
        double dy = pt.y() - this->pose.position.y;

        double c = std::cos(-yaw);
        double s = std::sin(-yaw);

        double rot_x = dx * c - dy * s;
        double rot_y = dx * s + dy * c;

        int x = static_cast<int>(rot_x / this->grid_resolution_) + this->grid_size_ / 2;
        int y = static_cast<int>(rot_y / this->grid_resolution_) + this->grid_size_ / 2;

        if (x >= 0 && x < this->grid_size_ && y >= 0 && y < this->grid_size_)
        {
            return cv::Point(x, y);
        }
        else
        {
            return std::nullopt;
        }
    }

    void OccupancyGrid::readLaneletMap()
    {
        // Create a projector using your origin
        lanelet::Origin origin({this->origin_x, this->origin_y, 0.0});

        lanelet::projection::UtmProjector projector = lanelet::projection::UtmProjector(origin);

        try {
            this->lanelet_map_ = lanelet::load(this->osm_path, projector);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Exception while loading lanelet map: %s", e.what());
            this->lanelet_map_ = nullptr;
        }

        if (this->lanelet_map_ == nullptr)
        {
            if (this->camera_mode_)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to load lanelet map, but continuing in Camera Mode.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load lanelet map");
                rclcpp::shutdown(); // BE CAREFUL FOR CONTAINERIZED NODES
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Lanelet map is loaded successfully!");
        }
    }

} // namespace itusct

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(itusct::OccupancyGrid)