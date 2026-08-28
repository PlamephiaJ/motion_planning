#include "motion_planning/RRT.hpp"

#include "motion_planning/FileHandler.hpp"
#include "motion_planning/occupancy_grid.hpp"
#include "motion_planning/path_tracking.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <utility>

RRT::RRT()
    : rclcpp::Node("rrt_node")
{
    load_parameters();
    load_global_waypoints();
    initialize_algorithm_modules();
    initialize_ros_interfaces();

    RCLCPP_INFO(this->get_logger(), "RRT motion-planning node started.");
    RCLCPP_INFO(
        this->get_logger(),
        "Control modules: steering=%s lqg=%s speed=%s curvature_limiter=%s "
        "blocked_path_limiter=%s.",
        steering_controller_type_.c_str(), lqg_enabled_ ? "on" : "off",
        speed_controller_type_.c_str(),
        curvature_speed_limiter_enabled_ ? "on" : "off",
        blocked_path_speed_limiter_enabled_ ? "on" : "off");
    if (is_vehicle_enabled_)
    {
        RCLCPP_WARN(this->get_logger(), "Vehicle motion is enabled on launch.");
    }
    else
    {
        stop_vehicle();
        RCLCPP_INFO(
            this->get_logger(),
            "Vehicle stopped. Publish 'start' to %s (single vehicle) or %s "
            "(all vehicles) to enable motion.",
            control_topic_.c_str(), fleet_control_topic_.c_str());
    }
}

RRT::~RRT()
{
    RCLCPP_INFO(this->get_logger(), "Exiting RRT node.");
}

void RRT::load_parameters()
{
    this->declare_parameter("MARGIN", map_inflation_margin_);
    map_inflation_margin_ = this->get_parameter("MARGIN").as_double();
    if (map_inflation_margin_ < 0.0)
    {
        throw std::invalid_argument("Bad configuration. MARGIN must be >= 0.");
    }

    this->declare_parameter("DISTANCE_GOAL_AHEAD", goal_ahead_distance_);
    goal_ahead_distance_ = this->get_parameter("DISTANCE_GOAL_AHEAD").as_double();
    if (goal_ahead_distance_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. DISTANCE_GOAL_AHEAD must be > 0.");
    }

    this->declare_parameter("SCAN_RANGE", scan_range_);
    scan_range_ = this->get_parameter("SCAN_RANGE").as_double();
    if (scan_range_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. SCAN_RANGE must be > 0.");
    }

    this->declare_parameter(
        "DETECTED_OBS_MARGIN", detected_obstacle_margin_);
    detected_obstacle_margin_ =
        this->get_parameter("DETECTED_OBS_MARGIN").as_double();
    if (detected_obstacle_margin_ < 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. DETECTED_OBS_MARGIN must be >= 0.");
    }

    this->declare_parameter(
        "DYNAMIC_OBSTACLE_PERSISTENCE", dynamic_obstacle_persistence_);
    dynamic_obstacle_persistence_ = this->get_parameter(
        "DYNAMIC_OBSTACLE_PERSISTENCE").as_double();
    this->declare_parameter(
        "DYNAMIC_MAP_UPDATE_PERIOD", dynamic_map_update_period_);
    dynamic_map_update_period_ = this->get_parameter(
        "DYNAMIC_MAP_UPDATE_PERIOD").as_double();
    if (dynamic_obstacle_persistence_ <= 0.0 ||
        dynamic_map_update_period_ <= 0.0 ||
        dynamic_map_update_period_ > dynamic_obstacle_persistence_)
    {
        throw std::invalid_argument(
            "Bad configuration. Dynamic-obstacle timing must satisfy "
            "0 < update period <= persistence.");
    }
    this->declare_parameter(
        "PLANNING_UPDATE_PERIOD", planning_update_period_);
    planning_update_period_ = this->get_parameter(
        "PLANNING_UPDATE_PERIOD").as_double();
    if (planning_update_period_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. PLANNING_UPDATE_PERIOD must be > 0.");
    }

    this->declare_parameter("MIN_RRT_ITERATIONS", minimum_rrt_iterations_);
    minimum_rrt_iterations_ =
        this->get_parameter("MIN_RRT_ITERATIONS").as_int();
    if (minimum_rrt_iterations_ <= 0)
    {
        throw std::invalid_argument(
            "Bad configuration. MIN_RRT_ITERATIONS must be > 0.");
    }

    this->declare_parameter("MAX_RRT_ITERATIONS", maximum_rrt_iterations_);
    maximum_rrt_iterations_ =
        this->get_parameter("MAX_RRT_ITERATIONS").as_int();
    if (maximum_rrt_iterations_ < minimum_rrt_iterations_)
    {
        throw std::invalid_argument(
            "Bad configuration. MAX_RRT_ITERATIONS must be >= "
            "MIN_RRT_ITERATIONS.");
    }

    this->declare_parameter("STD", sample_standard_deviation_);
    sample_standard_deviation_ = this->get_parameter("STD").as_double();
    if (sample_standard_deviation_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. STD must be > 0.");
    }

    this->declare_parameter("STEP_SIZE", step_size_);
    step_size_ = this->get_parameter("STEP_SIZE").as_double();
    if (step_size_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. STEP_SIZE must be > 0.");
    }

    this->declare_parameter("NEAR_RANGE", near_range_);
    near_range_ = this->get_parameter("NEAR_RANGE").as_double();
    if (near_range_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. NEAR_RANGE must be > 0.");
    }

    this->declare_parameter("GOAL_TOLERANCE", goal_tolerance_);
    goal_tolerance_ = this->get_parameter("GOAL_TOLERANCE").as_double();
    if (goal_tolerance_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. GOAL_TOLERANCE must be > 0.");
    }

    this->declare_parameter("GOAL_SAMPLE_RATE", goal_sample_rate_);
    goal_sample_rate_ = this->get_parameter("GOAL_SAMPLE_RATE").as_double();
    if (goal_sample_rate_ < 0.0 || goal_sample_rate_ > 1.0)
    {
        throw std::invalid_argument(
            "Bad configuration. GOAL_SAMPLE_RATE must be in [0, 1].");
    }

    this->declare_parameter("RRT_WAYPOINT_INTERVAL", rrt_waypoint_interval_);
    rrt_waypoint_interval_ =
        this->get_parameter("RRT_WAYPOINT_INTERVAL").as_double();
    if (rrt_waypoint_interval_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. RRT_WAYPOINT_INTERVAL must be > 0.");
    }

    this->declare_parameter(
        "OPTIMAL_REJOIN_DISTANCE", optimal_rejoin_distance_);
    optimal_rejoin_distance_ =
        this->get_parameter("OPTIMAL_REJOIN_DISTANCE").as_double();
    if (optimal_rejoin_distance_ < 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. OPTIMAL_REJOIN_DISTANCE must be >= 0.");
    }

    this->declare_parameter(
        "OPTIMAL_REJOIN_CLEAR_TIME", optimal_rejoin_clear_time_);
    optimal_rejoin_clear_time_ =
        this->get_parameter("OPTIMAL_REJOIN_CLEAR_TIME").as_double();
    if (optimal_rejoin_clear_time_ < 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. OPTIMAL_REJOIN_CLEAR_TIME must be >= 0.");
    }

    this->declare_parameter(
        "PROGRESS_SEARCH_BACKWARD", progress_search_backward_);
    progress_search_backward_ =
        this->get_parameter("PROGRESS_SEARCH_BACKWARD").as_double();
    this->declare_parameter(
        "PROGRESS_SEARCH_FORWARD", progress_search_forward_);
    progress_search_forward_ =
        this->get_parameter("PROGRESS_SEARCH_FORWARD").as_double();
    this->declare_parameter(
        "PROJECTION_FALLBACK_DISTANCE", projection_fallback_distance_);
    projection_fallback_distance_ =
        this->get_parameter("PROJECTION_FALLBACK_DISTANCE").as_double();
    if (progress_search_backward_ < 0.0 || progress_search_forward_ < 0.0 ||
        projection_fallback_distance_ < 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. Progress search distances must be >= 0.");
    }

    this->declare_parameter(
        "STEERING_CONTROLLER_TYPE", steering_controller_type_);
    steering_controller_type_ = this->get_parameter(
        "STEERING_CONTROLLER_TYPE").as_string();
    if (steering_controller_type_ != "legacy_pure_pursuit" &&
        steering_controller_type_ != "pure_pursuit")
    {
        throw std::invalid_argument(
            "Bad configuration. STEERING_CONTROLLER_TYPE must be "
            "legacy_pure_pursuit or pure_pursuit.");
    }

    this->declare_parameter("DISTANCE_LOOK_AHEAD", lookahead_distance_);
    lookahead_distance_ =
        this->get_parameter("DISTANCE_LOOK_AHEAD").as_double();
    if (steering_controller_type_ == "legacy_pure_pursuit" &&
        lookahead_distance_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. DISTANCE_LOOK_AHEAD must be > 0.");
    }

    this->declare_parameter("PID_P", pursuit_gain_);
    pursuit_gain_ = this->get_parameter("PID_P").as_double();
    if (steering_controller_type_ == "legacy_pure_pursuit" &&
        pursuit_gain_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. PID_P must be > 0.");
    }

    this->declare_parameter("PURE_PURSUIT_WHEELBASE", pure_pursuit_config_.wheelbase);
    pure_pursuit_config_.wheelbase = this->get_parameter(
        "PURE_PURSUIT_WHEELBASE").as_double();
    this->declare_parameter("PURE_PURSUIT_LOOKAHEAD", pure_pursuit_config_.lookahead);
    pure_pursuit_config_.lookahead = this->get_parameter(
        "PURE_PURSUIT_LOOKAHEAD").as_double();
    this->declare_parameter(
        "PURE_PURSUIT_LOOKAHEAD_SPEED_GAIN",
        pure_pursuit_config_.lookahead_speed_gain);
    pure_pursuit_config_.lookahead_speed_gain = this->get_parameter(
        "PURE_PURSUIT_LOOKAHEAD_SPEED_GAIN").as_double();
    this->declare_parameter(
        "PURE_PURSUIT_MAX_LOOKAHEAD", pure_pursuit_config_.max_lookahead);
    pure_pursuit_config_.max_lookahead = this->get_parameter(
        "PURE_PURSUIT_MAX_LOOKAHEAD").as_double();
    this->declare_parameter(
        "PURE_PURSUIT_MAX_STEERING", pure_pursuit_config_.max_steering);
    pure_pursuit_config_.max_steering = this->get_parameter(
        "PURE_PURSUIT_MAX_STEERING").as_double();

    this->declare_parameter("LQG_ENABLED", lqg_enabled_);
    lqg_enabled_ = this->get_parameter("LQG_ENABLED").as_bool();
    this->declare_parameter("LQG_WHEELBASE", lqg_config_.wheelbase);
    lqg_config_.wheelbase = this->get_parameter("LQG_WHEELBASE").as_double();
    this->declare_parameter("LQG_DT", lqg_config_.dt);
    lqg_config_.dt = this->get_parameter("LQG_DT").as_double();
    this->declare_parameter(
        "LQG_CROSS_TRACK_WEIGHT", lqg_config_.cross_track_weight);
    lqg_config_.cross_track_weight = this->get_parameter(
        "LQG_CROSS_TRACK_WEIGHT").as_double();
    this->declare_parameter(
        "LQG_HEADING_WEIGHT", lqg_config_.heading_weight);
    lqg_config_.heading_weight = this->get_parameter(
        "LQG_HEADING_WEIGHT").as_double();
    this->declare_parameter(
        "LQG_STEERING_WEIGHT", lqg_config_.steering_weight);
    lqg_config_.steering_weight = this->get_parameter(
        "LQG_STEERING_WEIGHT").as_double();
    this->declare_parameter("LQG_AUTHORITY", lqg_config_.authority);
    lqg_config_.authority = this->get_parameter("LQG_AUTHORITY").as_double();
    this->declare_parameter(
        "LQG_MAX_STEERING", lqg_config_.max_steering);
    lqg_config_.max_steering = this->get_parameter(
        "LQG_MAX_STEERING").as_double();
    this->declare_parameter(
        "LQG_MAX_STEERING_RATE", lqg_config_.max_steering_rate);
    lqg_config_.max_steering_rate = this->get_parameter(
        "LQG_MAX_STEERING_RATE").as_double();
    this->declare_parameter(
        "LQG_MINIMUM_MODEL_SPEED", lqg_config_.minimum_model_speed);
    lqg_config_.minimum_model_speed = this->get_parameter(
        "LQG_MINIMUM_MODEL_SPEED").as_double();

    this->declare_parameter("SPEED_CONTROLLER_TYPE", speed_controller_type_);
    speed_controller_type_ = this->get_parameter(
        "SPEED_CONTROLLER_TYPE").as_string();
    if (speed_controller_type_ != "steering_band" &&
        speed_controller_type_ != "trajectory")
    {
        throw std::invalid_argument(
            "Bad configuration. SPEED_CONTROLLER_TYPE must be "
            "steering_band or trajectory.");
    }

    this->declare_parameter(
        "SPEED_LOW_STEERING_THRESHOLD_DEGREES",
        steering_band_speed_config_.low_steering_threshold_degrees);
    steering_band_speed_config_.low_steering_threshold_degrees = this->get_parameter(
        "SPEED_LOW_STEERING_THRESHOLD_DEGREES").as_double();
    this->declare_parameter(
        "SPEED_MEDIUM_STEERING_THRESHOLD_DEGREES",
        steering_band_speed_config_.medium_steering_threshold_degrees);
    steering_band_speed_config_.medium_steering_threshold_degrees = this->get_parameter(
        "SPEED_MEDIUM_STEERING_THRESHOLD_DEGREES").as_double();
    if (speed_controller_type_ == "steering_band" &&
        (steering_band_speed_config_.low_steering_threshold_degrees < 0.0 ||
        steering_band_speed_config_.medium_steering_threshold_degrees <=
        steering_band_speed_config_.low_steering_threshold_degrees))
    {
        throw std::invalid_argument(
            "Bad configuration. Speed steering thresholds must satisfy "
            "0 <= low < medium.");
    }

    this->declare_parameter(
        "SPEED_STRAIGHT", steering_band_speed_config_.straight_speed);
    steering_band_speed_config_.straight_speed =
        this->get_parameter("SPEED_STRAIGHT").as_double();
    this->declare_parameter(
        "SPEED_MEDIUM_TURN", steering_band_speed_config_.medium_turn_speed);
    steering_band_speed_config_.medium_turn_speed =
        this->get_parameter("SPEED_MEDIUM_TURN").as_double();
    this->declare_parameter(
        "SPEED_SHARP_TURN", steering_band_speed_config_.sharp_turn_speed);
    steering_band_speed_config_.sharp_turn_speed =
        this->get_parameter("SPEED_SHARP_TURN").as_double();
    if (speed_controller_type_ == "steering_band" &&
        (steering_band_speed_config_.sharp_turn_speed <= 0.0 ||
        steering_band_speed_config_.medium_turn_speed <
        steering_band_speed_config_.sharp_turn_speed ||
        steering_band_speed_config_.straight_speed <
        steering_band_speed_config_.medium_turn_speed))
    {
        throw std::invalid_argument(
            "Bad configuration. Speeds must satisfy "
            "straight >= medium turn >= sharp turn > 0.");
    }

    this->declare_parameter(
        "TRAJECTORY_SPEED_MAX", trajectory_speed_config_.max_speed);
    trajectory_speed_config_.max_speed = this->get_parameter(
        "TRAJECTORY_SPEED_MAX").as_double();
    this->declare_parameter(
        "TRAJECTORY_SPEED_MINIMUM_STEERING_SCALE",
        trajectory_speed_config_.minimum_speed_scale);
    trajectory_speed_config_.minimum_speed_scale = this->get_parameter(
        "TRAJECTORY_SPEED_MINIMUM_STEERING_SCALE").as_double();
    this->declare_parameter(
        "TRAJECTORY_SPEED_PREVIEW_DISTANCE",
        trajectory_speed_config_.speed_preview_distance);
    trajectory_speed_config_.speed_preview_distance = this->get_parameter(
        "TRAJECTORY_SPEED_PREVIEW_DISTANCE").as_double();
    this->declare_parameter(
        "TRAJECTORY_SPEED_MAX_DECELERATION",
        trajectory_speed_config_.max_deceleration);
    trajectory_speed_config_.max_deceleration = this->get_parameter(
        "TRAJECTORY_SPEED_MAX_DECELERATION").as_double();
    this->declare_parameter(
        "TRAJECTORY_SPEED_CROSS_TRACK_ERROR_GAIN",
        trajectory_speed_config_.cross_track_error_gain);
    trajectory_speed_config_.cross_track_error_gain = this->get_parameter(
        "TRAJECTORY_SPEED_CROSS_TRACK_ERROR_GAIN").as_double();
    this->declare_parameter(
        "TRAJECTORY_SPEED_MINIMUM_TRACKING_SCALE",
        trajectory_speed_config_.minimum_tracking_speed_scale);
    trajectory_speed_config_.minimum_tracking_speed_scale = this->get_parameter(
        "TRAJECTORY_SPEED_MINIMUM_TRACKING_SCALE").as_double();
    this->declare_parameter(
        "TRAJECTORY_SPEED_MAX_STEERING",
        trajectory_speed_config_.max_steering);
    trajectory_speed_config_.max_steering = this->get_parameter(
        "TRAJECTORY_SPEED_MAX_STEERING").as_double();

    this->declare_parameter(
        "CURVATURE_SPEED_LIMITER_ENABLED",
        curvature_speed_limiter_enabled_);
    curvature_speed_limiter_enabled_ = this->get_parameter(
        "CURVATURE_SPEED_LIMITER_ENABLED").as_bool();
    this->declare_parameter(
        "CURVATURE_MAX_LATERAL_ACCELERATION",
        curvature_limiter_config_.max_lateral_acceleration);
    curvature_limiter_config_.max_lateral_acceleration = this->get_parameter(
        "CURVATURE_MAX_LATERAL_ACCELERATION").as_double();

    this->declare_parameter(
        "BLOCKED_PATH_SPEED_LIMITER_ENABLED",
        blocked_path_speed_limiter_enabled_);
    blocked_path_speed_limiter_enabled_ = this->get_parameter(
        "BLOCKED_PATH_SPEED_LIMITER_ENABLED").as_bool();

    this->declare_parameter(
        "BLOCKED_PATH_STOP_DISTANCE", blocked_path_stop_distance_);
    blocked_path_stop_distance_ = this->get_parameter(
        "BLOCKED_PATH_STOP_DISTANCE").as_double();
    this->declare_parameter(
        "BLOCKED_PATH_SPEED_GAIN", blocked_path_speed_gain_);
    blocked_path_speed_gain_ = this->get_parameter(
        "BLOCKED_PATH_SPEED_GAIN").as_double();
    if (blocked_path_speed_limiter_enabled_ &&
        (blocked_path_stop_distance_ < 0.0 ||
        blocked_path_speed_gain_ <= 0.0))
    {
        throw std::invalid_argument(
            "Bad configuration. BLOCKED_PATH_STOP_DISTANCE must be >= 0 "
            "and BLOCKED_PATH_SPEED_GAIN must be > 0.");
    }

    this->declare_parameter(
        "VISUALIZATION_PRIMARY_COLOR", visualization_primary_color_);
    visualization_primary_color_ = this->get_parameter(
        "VISUALIZATION_PRIMARY_COLOR").as_double_array();
    this->declare_parameter(
        "VISUALIZATION_ACCENT_COLOR", visualization_accent_color_);
    visualization_accent_color_ = this->get_parameter(
        "VISUALIZATION_ACCENT_COLOR").as_double_array();
    const auto valid_color = [](const std::vector<double>& color)
        {
            return color.size() == 3 && std::all_of(
                color.begin(), color.end(),
                [](const double channel)
                {
                    return channel >= 0.0 && channel <= 1.0;
                });
        };
    if (!valid_color(visualization_primary_color_) ||
        !valid_color(visualization_accent_color_))
    {
        throw std::invalid_argument(
            "Bad configuration. Visualization colors must be RGB arrays "
            "with three values in [0, 1].");
    }

    this->declare_parameter("odometry_topic", odometry_topic_);
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    this->declare_parameter("map_topic", map_topic_);
    map_topic_ = this->get_parameter("map_topic").as_string();
    this->declare_parameter("scan_topic", scan_topic_);
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    this->declare_parameter("dynamic_map_topic", dynamic_map_topic_);
    dynamic_map_topic_ = this->get_parameter("dynamic_map_topic").as_string();
    this->declare_parameter("drive_topic", drive_topic_);
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    this->declare_parameter("control_topic", control_topic_);
    control_topic_ = this->get_parameter("control_topic").as_string();
    this->declare_parameter("fleet_control_topic", fleet_control_topic_);
    fleet_control_topic_ =
        this->get_parameter("fleet_control_topic").as_string();
    if (odometry_topic_.empty() || map_topic_.empty() || scan_topic_.empty() ||
        dynamic_map_topic_.empty() || drive_topic_.empty() ||
        control_topic_.empty() || fleet_control_topic_.empty())
    {
        throw std::invalid_argument(
            "Bad configuration. ROS topic names must not be empty.");
    }

    this->declare_parameter("map_frame", map_frame_);
    map_frame_ = this->get_parameter("map_frame").as_string();
    this->declare_parameter("laser_frame", laser_frame_);
    laser_frame_ = this->get_parameter("laser_frame").as_string();
    this->declare_parameter("vehicle_frame", vehicle_frame_);
    vehicle_frame_ = this->get_parameter("vehicle_frame").as_string();
    if (map_frame_.empty() || laser_frame_.empty() || vehicle_frame_.empty())
    {
        throw std::invalid_argument(
            "Bad configuration. TF frame names must not be empty.");
    }

    this->declare_parameter("start_on_launch", start_on_launch_);
    start_on_launch_ = this->get_parameter("start_on_launch").as_bool();
    is_vehicle_enabled_ = start_on_launch_;

    this->declare_parameter("waypoint_file_path", waypoint_file_path_);
    waypoint_file_path_ =
        this->get_parameter("waypoint_file_path").as_string();
}

void RRT::load_global_waypoints()
{
    CSVHandler csv_handler(waypoint_file_path_);
    reference_trajectory::Path samples;
    try
    {
        const auto records =
            csv_handler.read_waypoint_and_speed_list_from_csv();
        samples.reserve(records.size());
        for (const auto& record : records)
        {
            samples.push_back({record.position, record.speed});
        }
    }
    catch (const std::exception& error)
    {
        if (speed_controller_type_ == "trajectory")
        {
            throw std::invalid_argument(
                std::string("Trajectory speed controller requires x,y,speed CSV: ") +
                error.what());
        }
        const auto positions = csv_handler.read_waypoint_list_from_csv();
        samples.reserve(positions.size());
        for (const auto& position : positions)
        {
            samples.push_back(
                {position, steering_band_speed_config_.straight_speed});
        }
        RCLCPP_WARN(
            this->get_logger(),
            "Waypoint CSV has no usable speed column (%s); steering_band "
            "mode is filling paired nominal speeds with SPEED_STRAIGHT.",
            error.what());
    }
    reference_trajectory_ =
        std::make_unique<reference_trajectory::ReferenceTrajectory>(samples);
    global_waypoints_ = reference_trajectory_->positions();
}

void RRT::initialize_algorithm_modules()
{
    rrt_star::PlannerConfig planner_config;
    planner_config.minimum_iterations = minimum_rrt_iterations_;
    planner_config.maximum_iterations = maximum_rrt_iterations_;
    planner_config.sample_standard_deviation = sample_standard_deviation_;
    planner_config.step_size = step_size_;
    planner_config.near_radius = near_range_;
    planner_config.goal_tolerance = goal_tolerance_;
    planner_config.goal_sample_rate = goal_sample_rate_;
    planner_config.static_margin = map_inflation_margin_;
    planner_config.dynamic_margin = detected_obstacle_margin_;
    planner_ = std::make_unique<rrt_star::Planner>(planner_config);

    motion_control::LegacyPurePursuitConfig legacy_config;
    legacy_config.lookahead_distance = lookahead_distance_;
    legacy_config.proportional_gain = pursuit_gain_;
    legacy_config.max_steering = steering_limit_;
    if (steering_controller_type_ == "pure_pursuit")
    {
        pure_pursuit_controller_ =
            std::make_unique<motion_control::PurePursuitController>(
                pure_pursuit_config_);
    }
    else
    {
        legacy_pure_pursuit_controller_ =
            std::make_unique<motion_control::LegacyPurePursuitController>(
                legacy_config);
    }
    if (lqg_enabled_)
    {
        lqg_controller_ =
            std::make_unique<motion_control::LqgController>(lqg_config_);
    }
    if (speed_controller_type_ == "trajectory")
    {
        trajectory_speed_controller_ =
            std::make_unique<motion_control::TrajectorySpeedController>(
                trajectory_speed_config_);
    }
    else
    {
        steering_band_speed_controller_ =
            std::make_unique<motion_control::SteeringBandSpeedController>(
                steering_band_speed_config_);
    }
    if (curvature_speed_limiter_enabled_)
    {
        curvature_speed_limiter_ =
            std::make_unique<motion_control::CurvatureSpeedLimiter>(
                curvature_limiter_config_);
    }
    motion_control::BlockedPathSpeedLimiterConfig blocked_config;
    blocked_config.stop_distance = blocked_path_stop_distance_;
    blocked_config.gain = blocked_path_speed_gain_;
    if (blocked_path_speed_limiter_enabled_)
    {
        blocked_path_speed_limiter_ =
            std::make_unique<motion_control::BlockedPathSpeedLimiter>(
                blocked_config);
    }

    reference_path::ManagerConfig reference_config;
    reference_config.global_goal_distance = goal_ahead_distance_;
    reference_config.rejoin_distance = optimal_rejoin_distance_;
    reference_config.rejoin_clearance_time = optimal_rejoin_clear_time_;
    reference_config.progress_search_backward = progress_search_backward_;
    reference_config.progress_search_forward = progress_search_forward_;
    reference_config.projection_fallback_distance =
        projection_fallback_distance_;
    reference_manager_ = std::make_unique<reference_path::Manager>(
        global_waypoints_, reference_config);
}

void RRT::initialize_ros_interfaces()
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto normalize_frame = [](std::string& frame)
        {
            frame.erase(0, frame.find_first_not_of('/'));
        };
    normalize_frame(map_frame_);
    normalize_frame(laser_frame_);
    normalize_frame(vehicle_frame_);

    const auto map_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
        std::bind(&RRT::map_callback, this, std::placeholders::_1));
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 1,
        std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 1,
        std::bind(&RRT::odometry_callback, this, std::placeholders::_1));
    control_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        control_topic_, 10,
        std::bind(&RRT::control_callback, this, std::placeholders::_1));
    if (fleet_control_topic_ != control_topic_)
    {
        fleet_control_subscriber_ =
            this->create_subscription<std_msgs::msg::String>(
                fleet_control_topic_, 10,
                std::bind(
                    &RRT::control_callback, this, std::placeholders::_1));
    }

    dynamic_map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(dynamic_map_topic_, 1);
    path_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("path", 1);
    tree_node_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("tree_nodes", 1);
    tree_branch_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("tree_branches", 1);
    goal_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("goal", 10);
    const auto waypoint_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    waypoint_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "global_waypoints", waypoint_qos);
    lookahead_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("lookahead", 10);
    drive_publisher_ =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic_, 1);

    initialize_visualization();
    const auto planning_period = std::chrono::duration_cast<
        std::chrono::nanoseconds>(
            std::chrono::duration<double>(planning_update_period_));
    planning_timer_ = this->create_wall_timer(
        planning_period, std::bind(&RRT::planning_timer_callback, this));
}

void RRT::initialize_visualization()
{
    const auto make_color = [](const std::vector<double>& rgb)
        {
            std_msgs::msg::ColorRGBA color;
            color.r = static_cast<float>(rgb.at(0));
            color.g = static_cast<float>(rgb.at(1));
            color.b = static_cast<float>(rgb.at(2));
            color.a = 1.0;
            return color;
        };
    const std_msgs::msg::ColorRGBA primary =
        make_color(visualization_primary_color_);
    const std_msgs::msg::ColorRGBA accent =
        make_color(visualization_accent_color_);

    goal_visualizer_ = std::make_unique<MarkerVisualizer>(
        goal_publisher_, "goal", map_frame_, primary, 0.3,
        visualization_msgs::msg::Marker::SPHERE);
    lookahead_visualizer_ = std::make_unique<MarkerVisualizer>(
        lookahead_publisher_, "lookahead", map_frame_, accent, 0.2,
        visualization_msgs::msg::Marker::SPHERE);
    global_waypoints_visualizer_ = std::make_unique<PointsVisualizer>(
        waypoint_publisher_, "global_waypoints", map_frame_, primary, 0.08);
    for (const auto& waypoint : global_waypoints_)
    {
        global_waypoints_visualizer_->add_point(waypoint);
    }
    global_waypoints_visualizer_->publish_points(false);
    global_waypoints_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        [this]() {global_waypoints_visualizer_->publish_points(false);});

    tree_nodes_.header.frame_id = map_frame_;
    tree_branches_.header.frame_id = map_frame_;
    tree_nodes_.ns = "nodes";
    tree_branches_.ns = "branch";
    tree_nodes_.action = visualization_msgs::msg::Marker::ADD;
    tree_branches_.action = visualization_msgs::msg::Marker::ADD;
    tree_nodes_.pose.orientation.w = 1.0;
    tree_branches_.pose.orientation.w = 1.0;
    tree_nodes_.id = 5;
    tree_branches_.id = 6;
    tree_nodes_.type = visualization_msgs::msg::Marker::POINTS;
    tree_branches_.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_nodes_.scale.x = 0.05;
    tree_nodes_.scale.y = 0.05;
    tree_nodes_.scale.z = 0.05;
    tree_branches_.scale.x = 0.01;
    tree_nodes_.color = accent;
    tree_branches_.color = primary;
}

void RRT::control_callback(const std_msgs::msg::String::ConstSharedPtr message)
{
    if (message->data == "start")
    {
        is_vehicle_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "Vehicle motion enabled.");
    }
    else if (message->data == "stop")
    {
        is_vehicle_enabled_ = false;
        stop_vehicle();
        RCLCPP_INFO(
            this->get_logger(),
            "Vehicle stopped. Planning and visualization remain active.");
    }
    else
    {
        RCLCPP_WARN(
            this->get_logger(), "Invalid control command '%s'.",
            message->data.c_str());
    }
}

void RRT::map_callback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
{
    obstacle_map_.initialize(*message, map_inflation_margin_);
    reference_manager_->reset();
    obstacle_frames_.clear();
    last_dynamic_map_update_seconds_ = -1.0;
    map_subscriber_.reset();
    RCLCPP_INFO(this->get_logger(), "Initial map received and inflated.");
}

bool RRT::lookup_laser_transform()
{
    try
    {
        laser_to_map_ = tf_buffer_->lookupTransform(
            map_frame_, laser_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& error)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Required TF unavailable: %s", error.what());
        return false;
    }
    return true;
}

bool RRT::lookup_vehicle_transforms()
{
    try
    {
        vehicle_to_map_ = tf_buffer_->lookupTransform(
            map_frame_, vehicle_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& error)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Vehicle TF unavailable: %s", error.what());
        return false;
    }
    return true;
}

geometry_msgs::msg::Point RRT::laser_point_to_map(
    const geometry_msgs::msg::Point& laser_point) const
{
    geometry_msgs::msg::PointStamped source;
    geometry_msgs::msg::PointStamped target;
    source.point = laser_point;
    tf2::doTransform(source, target, laser_to_map_);
    return target.point;
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
    if (!obstacle_map_.initialized())
    {
        return;
    }

    const double now_seconds = this->get_clock()->now().seconds();
    if (last_dynamic_map_update_seconds_ >= 0.0 &&
        now_seconds < last_dynamic_map_update_seconds_)
    {
        obstacle_frames_.clear();
        last_dynamic_map_update_seconds_ = -1.0;
    }
    if (last_dynamic_map_update_seconds_ >= 0.0 &&
        now_seconds - last_dynamic_map_update_seconds_ <
        dynamic_map_update_period_)
    {
        return;
    }
    if (!lookup_laser_transform())
    {
        return;
    }

    TimedObstacleFrame current_frame;
    current_frame.stamp_seconds = now_seconds;
    for (const auto& laser_point :
         dynamic_obstacles::valid_hit_points(*message, scan_range_))
    {
        current_frame.points.emplace_back(laser_point_to_map(laser_point));
    }
    obstacle_frames_.emplace_back(std::move(current_frame));
    last_dynamic_map_update_seconds_ = now_seconds;

    const double oldest_allowed =
        now_seconds - dynamic_obstacle_persistence_;
    while (!obstacle_frames_.empty() &&
           obstacle_frames_.front().stamp_seconds < oldest_allowed)
    {
        obstacle_frames_.pop_front();
    }

    std::vector<geometry_msgs::msg::Point> active_observations;
    for (const auto& frame : obstacle_frames_)
    {
        active_observations.insert(
            active_observations.end(), frame.points.begin(), frame.points.end());
    }
    const auto rebuild_profile = obstacle_map_.rebuild_observations(
        active_observations, detected_obstacle_margin_);
    RCLCPP_DEBUG(
        this->get_logger(),
        "Dynamic-map rebuild: %.3f ms (%zu observations, %zu changed cells)",
        std::chrono::duration<double, std::milli>(rebuild_profile.total).count(),
        rebuild_profile.observation_count, rebuild_profile.changed_cell_count);
    dynamic_map_publisher_->publish(obstacle_map_.collision_map());
}

void RRT::log_reference_transition(
    const reference_path::Decision& decision)
{
    if (!decision.mode_changed)
    {
        return;
    }
    if (decision.mode == reference_path::Mode::rrt_detour)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Optimal trajectory is blocked; switching to RRT* detour mode.");
    }
    else
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Optimal trajectory clear and safely reachable (distance %.3f m); "
            "returning to optimal-reference mode.",
            decision.projection.distance);
    }
}

void RRT::odometry_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
    // Odometry is vehicle-state input only. Its pose is intentionally ignored;
    // localization is obtained exclusively from map -> base_link TF.
    current_speed_ = message->twist.twist.linear.x;
    current_yaw_rate_ = message->twist.twist.angular.z;
}

void RRT::planning_timer_callback()
{
    if (!obstacle_map_.initialized())
    {
        return;
    }
    if (!lookup_vehicle_transforms())
    {
        stop_vehicle();
        return;
    }

    geometry_msgs::msg::Pose global_pose;
    global_pose.position.x = vehicle_to_map_.transform.translation.x;
    global_pose.position.y = vehicle_to_map_.transform.translation.y;
    global_pose.position.z = vehicle_to_map_.transform.translation.z;
    global_pose.orientation = vehicle_to_map_.transform.rotation;
    update_global_pose(global_pose);
}

void RRT::update_global_pose(const geometry_msgs::msg::Pose& global_pose)
{
    current_global_pose_ = global_pose;

    const reference_path::Decision reference = reference_manager_->update(
        current_global_pose_.position, obstacle_map_.collision_map(),
        this->get_clock()->now().seconds());
    log_reference_transition(reference);
    visualize_goal(reference.global_goal);

    if (reference.mode == reference_path::Mode::optimal_reference)
    {
        const auto path_points = path_tracking::resample_polyline(
            reference.local_optimal_reference, rrt_waypoint_interval_);
        const auto path = reference_trajectory_->associate(
            path_points, reference.projection.progress,
            progress_search_backward_, progress_search_forward_,
            projection_fallback_distance_);
        publish_path_marker(path_points);
        follow_path(path, false);
        clear_tree_visualization();
        return;
    }

    const rrt_star::PlanResult plan = planner_->plan(
        {current_global_pose_.position.x, current_global_pose_.position.y},
        {reference.global_goal.x, reference.global_goal.y},
        obstacle_map_.collision_map(),
        obstacle_map_.base_map());
    const auto profile_ms = [](const std::chrono::nanoseconds duration)
    {
        return std::chrono::duration<double, std::milli>(duration).count();
    };
    RCLCPP_DEBUG(
        this->get_logger(),
        "RRT* profile ms: sampling=%.3f nearest=%.3f initial_collision=%.3f "
        "near=%.3f parent_collision=%.3f rewiring=%.3f total=%.3f",
        profile_ms(plan.profile.sampling), profile_ms(plan.profile.nearest),
        profile_ms(plan.profile.initial_collision), profile_ms(plan.profile.near),
        profile_ms(plan.profile.parent_collision),
        profile_ms(plan.profile.rewiring), profile_ms(plan.profile.total));

    if (!plan.success)
    {
        const bool start_occupied = occupancy_grid::is_xy_coord_occupied(
            obstacle_map_.collision_map(), current_global_pose_.position.x,
            current_global_pose_.position.y);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Could not find a path. Tree nodes: %zu, goal candidates: %zu, "
            "start occupied: %s, sampling failed: %s.",
            plan.tree.size(), plan.goal_candidate_count,
            start_occupied ? "true" : "false",
            plan.failure == rrt_star::PlanFailure::sampling_failed ?
                "true" : "false");
        visualize_tree(plan.tree);
        if (!follow_blocked_reference(reference.local_optimal_reference))
        {
            stop_vehicle();
        }
        return;
    }

    const auto raw_points = path_tracking::nodes_to_points(plan.path);
    const auto path_points = path_tracking::resample_polyline(
        raw_points, rrt_waypoint_interval_);
    const auto path = reference_trajectory_->associate(
        path_points, reference.projection.progress,
        progress_search_backward_, progress_search_forward_,
        projection_fallback_distance_);
    publish_path_marker(path_points);
    follow_path(path, true);
    visualize_tree(plan.tree);
}

bool RRT::follow_blocked_reference(
    const std::vector<geometry_msgs::msg::Point>& optimal_reference)
{
    const auto collision_distance =
        occupancy_grid::distance_to_first_collision(
            obstacle_map_.collision_map(), optimal_reference);
    if (!collision_distance)
    {
        return false;
    }

    const std::optional<double> speed_limit =
        blocked_path_speed_limiter_enabled_ ?
        std::optional<double>(blocked_path_speed_limiter_->compute(
            *collision_distance)) : std::nullopt;
    const auto path_points = path_tracking::resample_polyline(
        optimal_reference, rrt_waypoint_interval_);
    const auto path = reference_trajectory_->associate(
        path_points, std::nullopt, progress_search_backward_,
        progress_search_forward_, projection_fallback_distance_);
    publish_path_marker(path_points);
    follow_path(path, false, speed_limit);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No detour available; LiDAR-only blocked-path control: "
        "collision distance %.2f m, speed limit %.2f m/s.",
        *collision_distance, speed_limit.value_or(-1.0));
    return true;
}

void RRT::follow_path(
    const reference_trajectory::Path& path,
    const bool is_rrt_detour,
    const std::optional<double> speed_limit)
{
    if (path.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Cannot follow an empty path.");
        stop_vehicle();
        return;
    }

    motion_control::VehiclePose pose;
    pose.x = current_global_pose_.position.x;
    pose.y = current_global_pose_.position.y;
    pose.yaw = tf2::getYaw(current_global_pose_.orientation);
    motion_control::SteeringCommand steering_command =
        steering_controller_type_ == "pure_pursuit" ?
        pure_pursuit_controller_->compute(path, pose, current_speed_) :
        legacy_pure_pursuit_controller_->compute(path, pose);
    if (lqg_enabled_)
    {
        steering_command = lqg_controller_->compute(
            path, pose, current_speed_, steering_command);
    }

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = this->now();
    double commanded_speed = speed_controller_type_ == "trajectory" ?
        trajectory_speed_controller_->compute(
            path, pose, steering_command.steering) :
        steering_band_speed_controller_->compute(steering_command.steering);
    if (!steering_command.has_forward_target)
    {
        commanded_speed = 0.0;
    }
    if (is_rrt_detour && curvature_speed_limiter_enabled_)
    {
        commanded_speed = std::min(
            commanded_speed,
            curvature_speed_limiter_->compute(
                path, steering_command.target_index));
    }
    if (speed_limit)
    {
        commanded_speed = std::min(
            commanded_speed, std::max(0.0, *speed_limit));
    }
    command.drive.speed = static_cast<float>(commanded_speed);
    command.drive.steering_angle = steering_command.steering;
    command.drive.steering_angle_velocity = 1.0;
    last_commanded_steering_angle_ = steering_command.steering;
    if (is_vehicle_enabled_)
    {
        drive_publisher_->publish(command);
    }
    else
    {
        stop_vehicle();
    }

    geometry_msgs::msg::Pose lookahead_pose;
    lookahead_pose.orientation.w = 1.0;
    lookahead_pose.position = steering_command.target;
    lookahead_visualizer_->set_pose(lookahead_pose);
    lookahead_visualizer_->publish_marker();
}

void RRT::stop_vehicle()
{
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = this->now();
    command.drive.speed = 0.0;
    command.drive.steering_angle = 0.0;
    last_commanded_steering_angle_ = 0.0;
    if (lqg_controller_)
    {
        lqg_controller_->reset();
    }
    drive_publisher_->publish(command);
}

void RRT::visualize_goal(const geometry_msgs::msg::Point& goal)
{
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position = goal;
    goal_visualizer_->set_pose(pose);
    goal_visualizer_->publish_marker();
}

void RRT::publish_path_marker(
    const std::vector<geometry_msgs::msg::Point>& points)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.id = 20;
    marker.ns = "path";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.color.r = static_cast<float>(visualization_primary_color_.at(0));
    marker.color.g = static_cast<float>(visualization_primary_color_.at(1));
    marker.color.b = static_cast<float>(visualization_primary_color_.at(2));
    marker.color.a = 1.0;
    marker.points = points;
    path_publisher_->publish(marker);
}

void RRT::visualize_tree(const rrt_star::Tree& tree)
{
    tree_nodes_.points.clear();
    tree_branches_.points.clear();
    for (const auto& node : tree)
    {
        geometry_msgs::msg::Point parent;
        parent.x = node.x;
        parent.y = node.y;
        tree_nodes_.points.emplace_back(parent);
        for (const std::size_t child_index : node.children)
        {
            tree_branches_.points.emplace_back(parent);
            geometry_msgs::msg::Point child;
            child.x = tree.at(child_index).x;
            child.y = tree.at(child_index).y;
            tree_branches_.points.emplace_back(child);
        }
    }
    tree_branch_publisher_->publish(tree_branches_);
    tree_node_publisher_->publish(tree_nodes_);
}

void RRT::clear_tree_visualization()
{
    tree_nodes_.points.clear();
    tree_branches_.points.clear();
    tree_branch_publisher_->publish(tree_branches_);
    tree_node_publisher_->publish(tree_nodes_);
}
