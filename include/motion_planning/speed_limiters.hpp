#ifndef MOTION_PLANNING__SPEED_LIMITERS_HPP_
#define MOTION_PLANNING__SPEED_LIMITERS_HPP_

#include "motion_planning/reference_trajectory.hpp"

#include <cstddef>
#include <vector>

namespace motion_control
{

struct CurvatureSpeedLimiterConfig
{
    double max_lateral_acceleration = 5.0;
    double minimum_curvature = 1.0e-5;
};

/** fy-code curve_speed = sqrt(max_lateral_accel / abs(curvature)). */
class CurvatureSpeedLimiter
{
public:
    explicit CurvatureSpeedLimiter(
        const CurvatureSpeedLimiterConfig& config = {});
    double limit_for_curvature(double curvature) const;
    double compute(
        const reference_trajectory::Path& path,
        std::size_t target_index) const;
    static std::vector<double> curvatures(
        const reference_trajectory::Path& path);

private:
    CurvatureSpeedLimiterConfig config_;
};

struct BlockedPathSpeedLimiterConfig
{
    double stop_distance = 0.5;
    double gain = 2.0;
};

/** Existing LiDAR blocked-reference cap, exposed as a safety limiter module. */
class BlockedPathSpeedLimiter
{
public:
    explicit BlockedPathSpeedLimiter(
        const BlockedPathSpeedLimiterConfig& config = {});
    double compute(double collision_distance) const;

private:
    BlockedPathSpeedLimiterConfig config_;
};

}  // namespace motion_control

#endif  // MOTION_PLANNING__SPEED_LIMITERS_HPP_
