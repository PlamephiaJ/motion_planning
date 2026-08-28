#ifndef MOTION_PLANNING__REFERENCE_TRAJECTORY_HPP_
#define MOTION_PLANNING__REFERENCE_TRAJECTORY_HPP_

#include "motion_planning/optimal_trajectory.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <optional>
#include <vector>

namespace reference_trajectory
{

/** One spatial trajectory sample and its corresponding nominal speed. */
struct Sample
{
    geometry_msgs::msg::Point position;
    double nominal_speed = 0.0;
};

using Path = std::vector<Sample>;

/**
 * Closed reference trajectory with position/speed correspondence preserved.
 *
 * Consecutive duplicate positions are removed together with their speed, and
 * position and speed are interpolated by the same arc-length ratio.
 */
class ReferenceTrajectory
{
public:
    explicit ReferenceTrajectory(const Path& samples);

    const std::vector<geometry_msgs::msg::Point>& positions() const;
    double total_length() const;
    Sample sample_at(double progress) const;
    Path slice(double start_progress, double forward_distance) const;

    /**
     * Attach interpolated reference speeds to an arbitrary ordered local path.
     * A progress hint keeps RRT detour samples associated with the same local
     * portion of a self-near or cyclic reference trajectory.
     */
    Path associate(
        const std::vector<geometry_msgs::msg::Point>& points,
        std::optional<double> progress_hint = std::nullopt,
        double backward_search_distance = 2.0,
        double forward_search_distance = 8.0,
        double fallback_distance = 2.5) const;

private:
    std::size_t segment_at(double normalized_progress) const;

    std::vector<geometry_msgs::msg::Point> positions_;
    std::vector<double> nominal_speeds_;
    std::vector<double> segment_lengths_;
    std::vector<double> cumulative_lengths_;
    optimal_trajectory::Trajectory geometry_;
};

/** Resample geometry and nominal speed together along an open local path. */
Path resample_path(const Path& path, double maximum_spacing);

/** Extract positions without changing their order. */
std::vector<geometry_msgs::msg::Point> positions(const Path& path);

}  // namespace reference_trajectory

#endif  // MOTION_PLANNING__REFERENCE_TRAJECTORY_HPP_
