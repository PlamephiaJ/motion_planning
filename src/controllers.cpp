#include "motion_planning/controllers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion_control
{

namespace
{

struct LocalPoint
{
    double x;
    double y;
    double distance;
};

LocalPoint to_vehicle_frame(
    const geometry_msgs::msg::Point& point, const VehiclePose& pose)
{
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    const double dx = point.x - pose.x;
    const double dy = point.y - pose.y;
    LocalPoint result;
    result.x = cosine * dx + sine * dy;
    result.y = -sine * dx + cosine * dy;
    result.distance = std::hypot(result.x, result.y);
    return result;
}

geometry_msgs::msg::Point point_at_distance(
    const reference_trajectory::Path& path, const double distance)
{
    double accumulated = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index)
    {
        const auto& start = path.at(index - 1).position;
        const auto& end = path.at(index).position;
        const double segment_length = std::hypot(
            end.x - start.x, end.y - start.y);
        if (segment_length <= std::numeric_limits<double>::epsilon())
        {
            continue;
        }
        if (accumulated + segment_length >= distance)
        {
            const double ratio = std::clamp(
                (distance - accumulated) / segment_length, 0.0, 1.0);
            geometry_msgs::msg::Point target;
            target.x = start.x + ratio * (end.x - start.x);
            target.y = start.y + ratio * (end.y - start.y);
            target.z = start.z + ratio * (end.z - start.z);
            return target;
        }
        accumulated += segment_length;
    }
    return path.back().position;
}

void validate_nonempty_path(const reference_trajectory::Path& path)
{
    if (path.empty())
    {
        throw std::invalid_argument("Controller path must not be empty");
    }
}

}  // namespace

LegacyPurePursuitController::LegacyPurePursuitController(
    const LegacyPurePursuitConfig& config)
    : config_(config)
{
    if (config_.lookahead_distance <= 0.0 ||
        config_.proportional_gain <= 0.0 || config_.max_steering <= 0.0)
    {
        throw std::invalid_argument(
            "Legacy Pure Pursuit parameters must be positive");
    }
}

SteeringCommand LegacyPurePursuitController::compute(
    const reference_trajectory::Path& path, const VehiclePose& pose) const
{
    validate_nonempty_path(path);
    const auto target = point_at_distance(path, config_.lookahead_distance);
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double lateral = -std::sin(pose.yaw) * dx +
        std::cos(pose.yaw) * dy;

    SteeringCommand result;
    const double steering = config_.proportional_gain * 2.0 * lateral /
        (config_.lookahead_distance * config_.lookahead_distance);
    result.steering = std::clamp(
        steering, -config_.max_steering, config_.max_steering);
    result.target = target;
    result.has_forward_target = true;
    double nearest_distance = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < path.size(); ++index)
    {
        const double distance = std::hypot(
            path.at(index).position.x - target.x,
            path.at(index).position.y - target.y);
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            result.target_index = index;
        }
    }
    return result;
}

PurePursuitController::PurePursuitController(const PurePursuitConfig& config)
    : config_(config)
{
    if (config_.wheelbase <= 0.0 || config_.lookahead <= 0.0 ||
        config_.lookahead_speed_gain < 0.0 ||
        config_.max_lookahead < config_.lookahead ||
        config_.max_steering <= 0.0)
    {
        throw std::invalid_argument("Invalid fy-code Pure Pursuit parameters");
    }
}

double PurePursuitController::lookahead_for_speed(
    const double current_speed) const
{
    return std::min(
        config_.max_lookahead,
        config_.lookahead + config_.lookahead_speed_gain *
        std::max(current_speed, 0.0));
}

SteeringCommand PurePursuitController::compute(
    const reference_trajectory::Path& path, const VehiclePose& pose,
    const double current_speed) const
{
    validate_nonempty_path(path);
    const double lookahead = lookahead_for_speed(current_speed);
    bool has_forward = false;
    bool has_beyond = false;
    double selected_distance = -1.0;
    double selected_lateral = 0.0;
    SteeringCommand result;
    for (std::size_t index = 0; index < path.size(); ++index)
    {
        const LocalPoint local = to_vehicle_frame(
            path.at(index).position, pose);
        if (local.x <= 0.02)
        {
            continue;
        }
        has_forward = true;
        if (local.distance >= lookahead)
        {
            if (!has_beyond || local.distance < selected_distance)
            {
                has_beyond = true;
                selected_distance = local.distance;
                selected_lateral = local.y;
                result.target_index = index;
            }
        }
        else if (!has_beyond && local.distance > selected_distance)
        {
            selected_distance = local.distance;
            selected_lateral = local.y;
            result.target_index = index;
        }
    }

    if (!has_forward)
    {
        result.target_index = path.size() - 1;
        result.target = path.back().position;
        return result;
    }
    const double distance_squared = std::max(
        selected_distance * selected_distance, 1.0e-6);
    result.steering = std::atan2(
        2.0 * config_.wheelbase * selected_lateral,
        distance_squared);
    result.steering = std::clamp(
        result.steering, -config_.max_steering, config_.max_steering);
    result.target = path.at(result.target_index).position;
    result.has_forward_target = true;
    return result;
}

SteeringBandSpeedController::SteeringBandSpeedController(
    const SteeringBandSpeedConfig& config)
    : config_(config)
{
    if (config_.low_steering_threshold_degrees < 0.0 ||
        config_.medium_steering_threshold_degrees <=
        config_.low_steering_threshold_degrees ||
        config_.sharp_turn_speed <= 0.0 ||
        config_.medium_turn_speed < config_.sharp_turn_speed ||
        config_.straight_speed < config_.medium_turn_speed)
    {
        throw std::invalid_argument("Invalid steering-band speed parameters");
    }
}

double SteeringBandSpeedController::compute(const double steering_angle) const
{
    constexpr double radians_to_degrees =
        180.0 / 3.14159265358979323846;
    const double degrees = std::abs(steering_angle) * radians_to_degrees;
    if (degrees <= config_.low_steering_threshold_degrees)
    {
        return config_.straight_speed;
    }
    if (degrees <= config_.medium_steering_threshold_degrees)
    {
        return config_.medium_turn_speed;
    }
    return config_.sharp_turn_speed;
}

TrajectorySpeedController::TrajectorySpeedController(
    const TrajectorySpeedConfig& config)
    : config_(config)
{
    if (config_.max_speed <= 0.0 || config_.max_steering <= 0.0 ||
        config_.speed_preview_distance <= 0.0 ||
        config_.max_deceleration <= 0.0 ||
        config_.cross_track_error_gain < 0.0 ||
        config_.minimum_speed_scale <= 0.0 ||
        config_.minimum_speed_scale > 1.0 ||
        config_.minimum_tracking_speed_scale <= 0.0 ||
        config_.minimum_tracking_speed_scale > 1.0)
    {
        throw std::invalid_argument("Invalid trajectory speed parameters");
    }
}

double TrajectorySpeedController::compute(
    const reference_trajectory::Path& path, const VehiclePose& pose,
    const double steering_angle) const
{
    validate_nonempty_path(path);
    double cross_track_error = std::numeric_limits<double>::max();
    double nearest_forward_distance = std::numeric_limits<double>::max();
    double nearest_forward_speed = 0.0;
    double minimum_braking_limit = std::numeric_limits<double>::max();
    bool has_forward = false;
    for (std::size_t index = 0; index < path.size(); ++index)
    {
        const LocalPoint local = to_vehicle_frame(
            path.at(index).position, pose);
        cross_track_error = std::min(
            cross_track_error, local.distance);
        if (local.x <= 0.02)
        {
            continue;
        }
        has_forward = true;
        if (local.distance < nearest_forward_distance)
        {
            nearest_forward_distance = local.distance;
            nearest_forward_speed = path.at(index).nominal_speed;
        }
        if (local.distance <= config_.speed_preview_distance)
        {
            const double point_limit = path.at(index).nominal_speed;
            const double braking_limit = std::sqrt(std::max(
                point_limit * point_limit +
                2.0 * config_.max_deceleration * local.distance, 0.0));
            minimum_braking_limit = std::min(
                minimum_braking_limit, braking_limit);
        }
    }
    if (!has_forward)
    {
        return 0.0;
    }

    const double preview_speed = std::min(
        nearest_forward_speed, minimum_braking_limit);

    const double steering_ratio = std::min(
        std::abs(steering_angle) / config_.max_steering, 1.0);
    const double speed_scale = std::max(
        config_.minimum_speed_scale, 1.0 - 0.55 * steering_ratio);
    const double tracking_scale = std::max(
        config_.minimum_tracking_speed_scale,
        1.0 - config_.cross_track_error_gain * cross_track_error);
    return std::clamp(preview_speed, 0.0, config_.max_speed) *
        speed_scale * tracking_scale;
}

}  // namespace motion_control
