#ifndef MOTION_PLANNING__CONTROLLERS_HPP_
#define MOTION_PLANNING__CONTROLLERS_HPP_

#include "motion_planning/reference_trajectory.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <cstddef>

namespace motion_control
{

struct VehiclePose
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct SteeringCommand
{
    double steering = 0.0;
    geometry_msgs::msg::Point target;
    std::size_t target_index = 0;
    bool has_forward_target = false;
};

/** Configuration for the original motion_planning Pure Pursuit behavior. */
struct LegacyPurePursuitConfig
{
    double lookahead_distance = 0.4;
    double proportional_gain = 0.25;
    double max_steering = 0.41;
};

/** Existing fixed-lookahead controller, retained as the default fallback. */
class LegacyPurePursuitController
{
public:
    explicit LegacyPurePursuitController(
        const LegacyPurePursuitConfig& config = {});
    SteeringCommand compute(
        const reference_trajectory::Path& path,
        const VehiclePose& pose) const;

private:
    LegacyPurePursuitConfig config_;
};

/** Configuration ported from fy-code PurePursuitConfig steering parameters. */
struct PurePursuitConfig
{
    double wheelbase = 0.33;
    double lookahead = 0.25;
    double lookahead_speed_gain = 0.05;
    double max_lookahead = 0.40;
    double max_steering = 0.42;
};

/** fy-code forward-sample Pure Pursuit with speed-dependent lookahead. */
class PurePursuitController
{
public:
    explicit PurePursuitController(const PurePursuitConfig& config = {});
    SteeringCommand compute(
        const reference_trajectory::Path& path,
        const VehiclePose& pose,
        double current_speed) const;
    double lookahead_for_speed(double current_speed) const;

private:
    PurePursuitConfig config_;
};

/** Existing three-band steering-angle speed policy. */
struct SteeringBandSpeedConfig
{
    double low_steering_threshold_degrees = 10.0;
    double medium_steering_threshold_degrees = 20.0;
    double straight_speed = 5.0;
    double medium_turn_speed = 3.0;
    double sharp_turn_speed = 1.5;
};

class SteeringBandSpeedController
{
public:
    explicit SteeringBandSpeedController(
        const SteeringBandSpeedConfig& config = {});
    double compute(double steering_angle) const;

private:
    SteeringBandSpeedConfig config_;
};

/** fy-code nominal-speed preview and braking-envelope parameters. */
struct TrajectorySpeedConfig
{
    double max_speed = 3.0;
    double minimum_speed_scale = 0.50;
    double speed_preview_distance = 2.5;
    double max_deceleration = 4.0;
    double cross_track_error_gain = 1.5;
    double minimum_tracking_speed_scale = 0.35;
    double max_steering = 0.42;
};

class TrajectorySpeedController
{
public:
    explicit TrajectorySpeedController(
        const TrajectorySpeedConfig& config = {});
    double compute(
        const reference_trajectory::Path& path,
        const VehiclePose& pose,
        double steering_angle) const;

private:
    TrajectorySpeedConfig config_;
};

}  // namespace motion_control

#endif  // MOTION_PLANNING__CONTROLLERS_HPP_
