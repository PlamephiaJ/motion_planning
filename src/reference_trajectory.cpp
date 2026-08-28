#include "motion_planning/reference_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace reference_trajectory
{

namespace
{

constexpr double epsilon = 1.0e-9;

double distance_between(
    const geometry_msgs::msg::Point& first,
    const geometry_msgs::msg::Point& second)
{
    return std::hypot(first.x - second.x, first.y - second.y);
}

std::vector<geometry_msgs::msg::Point> cleaned_positions(const Path& samples)
{
    std::vector<geometry_msgs::msg::Point> result;
    for (const auto& sample : samples)
    {
        if (!std::isfinite(sample.position.x) ||
            !std::isfinite(sample.position.y) ||
            !std::isfinite(sample.nominal_speed) ||
            sample.nominal_speed < 0.0)
        {
            throw std::invalid_argument(
                "Reference trajectory contains invalid position or speed");
        }
        if (result.empty() ||
            distance_between(result.back(), sample.position) > epsilon)
        {
            result.emplace_back(sample.position);
        }
    }
    if (result.size() > 1 &&
        distance_between(result.front(), result.back()) <= epsilon)
    {
        result.pop_back();
    }
    return result;
}

std::vector<double> cleaned_speeds(const Path& samples)
{
    std::vector<geometry_msgs::msg::Point> retained_positions;
    std::vector<double> result;
    for (const auto& sample : samples)
    {
        if (retained_positions.empty() ||
            distance_between(retained_positions.back(), sample.position) >
            epsilon)
        {
            retained_positions.emplace_back(sample.position);
            result.emplace_back(sample.nominal_speed);
        }
    }
    if (retained_positions.size() > 1 &&
        distance_between(
            retained_positions.front(), retained_positions.back()) <= epsilon)
    {
        result.pop_back();
    }
    return result;
}

}  // namespace

ReferenceTrajectory::ReferenceTrajectory(const Path& samples)
    : positions_(cleaned_positions(samples)),
      nominal_speeds_(cleaned_speeds(samples)), geometry_(positions_)
{
    if (nominal_speeds_.size() != positions_.size())
    {
        throw std::invalid_argument(
            "Reference trajectory position/speed arrays do not match");
    }

    cumulative_lengths_.emplace_back(0.0);
    double accumulated = 0.0;
    for (std::size_t index = 0; index < positions_.size(); ++index)
    {
        const double length = distance_between(
            positions_.at(index),
            positions_.at((index + 1) % positions_.size()));
        segment_lengths_.emplace_back(length);
        accumulated += length;
        cumulative_lengths_.emplace_back(accumulated);
    }
}

const std::vector<geometry_msgs::msg::Point>&
ReferenceTrajectory::positions() const
{
    return positions_;
}

double ReferenceTrajectory::total_length() const
{
    return geometry_.total_length();
}

std::size_t ReferenceTrajectory::segment_at(
    const double normalized_progress) const
{
    const auto upper = std::upper_bound(
        cumulative_lengths_.begin(), cumulative_lengths_.end(),
        normalized_progress);
    const std::size_t index = static_cast<std::size_t>(
        std::distance(cumulative_lengths_.begin(), upper) - 1);
    return std::min(index, segment_lengths_.size() - 1);
}

Sample ReferenceTrajectory::sample_at(const double progress) const
{
    const double normalized = geometry_.normalize_progress(progress);
    const std::size_t segment = segment_at(normalized);
    const double ratio =
        (normalized - cumulative_lengths_.at(segment)) /
        segment_lengths_.at(segment);
    const std::size_t next = (segment + 1) % positions_.size();
    Sample result;
    result.position = geometry_.point_at(normalized);
    result.nominal_speed = nominal_speeds_.at(segment) + ratio *
        (nominal_speeds_.at(next) - nominal_speeds_.at(segment));
    return result;
}

Path ReferenceTrajectory::slice(
    const double start_progress, const double forward_distance) const
{
    const auto path_positions = geometry_.slice(start_progress, forward_distance);
    return associate(
        path_positions, start_progress, 0.0,
        std::min(forward_distance + epsilon, total_length()),
        std::numeric_limits<double>::max());
}

Path ReferenceTrajectory::associate(
    const std::vector<geometry_msgs::msg::Point>& points,
    std::optional<double> progress_hint,
    const double backward_search_distance,
    const double forward_search_distance,
    const double fallback_distance) const
{
    Path result;
    result.reserve(points.size());
    for (const auto& point : points)
    {
        const auto projection = geometry_.project(
            point, progress_hint, backward_search_distance,
            forward_search_distance, fallback_distance);
        Sample sample;
        sample.position = point;
        sample.nominal_speed = sample_at(projection.progress).nominal_speed;
        result.emplace_back(sample);
        progress_hint = projection.progress;
    }
    return result;
}

Path resample_path(const Path& path, const double maximum_spacing)
{
    if (maximum_spacing <= 0.0)
    {
        throw std::invalid_argument("maximum_spacing must be positive");
    }
    if (path.size() < 2)
    {
        return path;
    }

    Path result;
    for (std::size_t index = 0; index + 1 < path.size(); ++index)
    {
        const auto& start = path.at(index);
        const auto& end = path.at(index + 1);
        result.emplace_back(start);
        const double length = distance_between(start.position, end.position);
        if (length < maximum_spacing)
        {
            continue;
        }
        const int segment_count = static_cast<int>(
            std::ceil(length / maximum_spacing));
        for (int sample_index = 1; sample_index < segment_count; ++sample_index)
        {
            const double ratio =
                static_cast<double>(sample_index) / segment_count;
            Sample sample;
            sample.position.x = start.position.x + ratio *
                (end.position.x - start.position.x);
            sample.position.y = start.position.y + ratio *
                (end.position.y - start.position.y);
            sample.position.z = start.position.z + ratio *
                (end.position.z - start.position.z);
            sample.nominal_speed = start.nominal_speed + ratio *
                (end.nominal_speed - start.nominal_speed);
            result.emplace_back(sample);
        }
    }
    result.emplace_back(path.back());
    return result;
}

std::vector<geometry_msgs::msg::Point> positions(const Path& path)
{
    std::vector<geometry_msgs::msg::Point> result;
    result.reserve(path.size());
    for (const auto& sample : path)
    {
        result.emplace_back(sample.position);
    }
    return result;
}

}  // namespace reference_trajectory
