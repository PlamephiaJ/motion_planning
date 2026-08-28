#ifndef MOTION_PLANNING__LQG_CONTROLLER_HPP_
#define MOTION_PLANNING__LQG_CONTROLLER_HPP_

#include "motion_planning/controllers.hpp"
#include "motion_planning/reference_trajectory.hpp"

#include <array>

namespace motion_control
{

/** Configuration ported from fy-code's two-state LQG strategy. */
struct LqgConfig
{
    double wheelbase = 0.33;
    double dt = 0.05;
    double cross_track_weight = 8.0;
    double heading_weight = 5.0;
    double steering_weight = 0.25;
    double authority = 0.25;
    double max_steering = 0.42;
    double max_steering_rate = 2.5;
    double minimum_model_speed = 0.5;
};

/**
 * Optional LQG steering correction layered over a configured base controller.
 *
 * The state is [cross-track error, heading error]. A two-state Kalman filter
 * estimates that state, a discrete LQR supplies feedback, and path curvature
 * supplies bicycle-model feedforward. The result is blended with `base` using
 * `authority` and rate-limited. Path positions and pose are in the same map
 * frame; speed is in m/s and steering values are in radians.
 *
 * Empty paths are rejected. Paths with fewer than two distinct points, or a
 * base command without a forward target, pass the base command through.
 */
class LqgController
{
public:
    explicit LqgController(const LqgConfig& config = {});

    SteeringCommand compute(
        const reference_trajectory::Path& path,
        const VehiclePose& pose,
        double current_speed,
        const SteeringCommand& base);

    /** Clear the Kalman estimate and previous steering after a commanded stop. */
    void reset();

private:
    LqgConfig config_;
    std::array<double, 2> estimate_{0.0, 0.0};
    std::array<double, 4> covariance_{0.1, 0.0, 0.0, 0.1};
    double previous_steering_ = 0.0;
};

}  // namespace motion_control

#endif  // MOTION_PLANNING__LQG_CONTROLLER_HPP_
