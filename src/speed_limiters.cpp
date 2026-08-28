#include "motion_planning/speed_limiters.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion_control
{

namespace
{

double local_curvature(
    const geometry_msgs::msg::Point& previous,
    const geometry_msgs::msg::Point& current,
    const geometry_msgs::msg::Point& following)
{
    const double before_x = current.x - previous.x;
    const double before_y = current.y - previous.y;
    const double after_x = following.x - current.x;
    const double after_y = following.y - current.y;
    const double across_x = following.x - previous.x;
    const double across_y = following.y - previous.y;
    const double denominator = std::hypot(before_x, before_y) *
        std::hypot(after_x, after_y) *
        std::max(std::hypot(across_x, across_y), 1.0e-9);
    return 2.0 * (before_x * after_y - before_y * after_x) /
        std::max(denominator, 1.0e-9);
}

}  // namespace

CurvatureSpeedLimiter::CurvatureSpeedLimiter(
    const CurvatureSpeedLimiterConfig& config)
    : config_(config)
{
    if (config_.max_lateral_acceleration <= 0.0 ||
        config_.minimum_curvature <= 0.0)
    {
        throw std::invalid_argument(
            "Curvature speed limiter parameters must be positive");
    }
}

double CurvatureSpeedLimiter::limit_for_curvature(
    const double curvature) const
{
    return std::sqrt(
        config_.max_lateral_acceleration /
        std::max(std::abs(curvature), config_.minimum_curvature));
}

std::vector<double> CurvatureSpeedLimiter::curvatures(
    const reference_trajectory::Path& path)
{
    std::vector<double> result(path.size(), 0.0);
    if (path.size() < 3)
    {
        return result;
    }
    for (std::size_t index = 1; index + 1 < path.size(); ++index)
    {
        result.at(index) = local_curvature(
            path.at(index - 1).position, path.at(index).position,
            path.at(index + 1).position);
    }
    result.front() = result.at(1);
    result.back() = result.at(result.size() - 2);
    return result;
}

double CurvatureSpeedLimiter::compute(
    const reference_trajectory::Path& path,
    const std::size_t target_index) const
{
    if (path.empty())
    {
        return std::numeric_limits<double>::infinity();
    }
    if (path.size() < 3)
    {
        return limit_for_curvature(0.0);
    }
    const std::size_t center = std::clamp(
        target_index, std::size_t{1}, path.size() - 2);
    return limit_for_curvature(local_curvature(
        path.at(center - 1).position, path.at(center).position,
        path.at(center + 1).position));
}

BlockedPathSpeedLimiter::BlockedPathSpeedLimiter(
    const BlockedPathSpeedLimiterConfig& config)
    : config_(config)
{
    if (config_.stop_distance < 0.0 || config_.gain <= 0.0)
    {
        throw std::invalid_argument(
            "Blocked-path stop distance must be non-negative and gain positive");
    }
}

double BlockedPathSpeedLimiter::compute(
    const double collision_distance) const
{
    if (collision_distance < 0.0)
    {
        throw std::invalid_argument(
            "Blocked-path collision distance must be non-negative");
    }
    return std::max(
        0.0, (collision_distance - config_.stop_distance) * config_.gain);
}

}  // namespace motion_control
