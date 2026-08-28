#include "motion_planning/lqg_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace motion_control
{

namespace
{

using Matrix2 = std::array<double, 4>;

constexpr std::size_t matrix_index(
    const std::size_t row, const std::size_t column)
{
    return row * 2 + column;
}

double wrap_angle(const double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double distance_squared(
    const geometry_msgs::msg::Point& point, const VehiclePose& pose)
{
    const double dx = pose.x - point.x;
    const double dy = pose.y - point.y;
    return dx * dx + dy * dy;
}

std::optional<double> reference_heading(
    const reference_trajectory::Path& path, const std::size_t center)
{
    for (std::size_t offset = 1; offset < path.size(); ++offset)
    {
        const std::size_t before = center > offset ? center - offset : 0;
        const std::size_t after = std::min(center + offset, path.size() - 1);
        const auto& start = path.at(before).position;
        const auto& end = path.at(after).position;
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        if (std::hypot(dx, dy) > 1.0e-9)
        {
            return std::atan2(dy, dx);
        }
        if (before == 0 && after + 1 == path.size())
        {
            break;
        }
    }
    return std::nullopt;
}

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
        std::hypot(after_x, after_y) * std::hypot(across_x, across_y);
    if (denominator <= 1.0e-9)
    {
        return 0.0;
    }
    return 2.0 * (before_x * after_y - before_y * after_x) /
        denominator;
}

double reference_curvature(
    const reference_trajectory::Path& path, const std::size_t center)
{
    if (path.size() < 3)
    {
        return 0.0;
    }
    const std::size_t curvature_center = std::clamp(
        center, std::size_t{1}, path.size() - 2);
    return local_curvature(
        path.at(curvature_center - 1).position,
        path.at(curvature_center).position,
        path.at(curvature_center + 1).position);
}

Matrix2 discrete_riccati(
    const double speed_dt, const double input_gain,
    const double cross_track_weight, const double heading_weight,
    const double steering_weight)
{
    // Iterating the discrete algebraic Riccati equation avoids adding a
    // heavyweight linear-algebra dependency for this fixed 2x2 system.
    Matrix2 riccati{
        cross_track_weight, 0.0, 0.0, heading_weight};
    constexpr std::size_t maximum_iterations = 1000;
    constexpr double tolerance = 1.0e-12;
    for (std::size_t iteration = 0;
        iteration < maximum_iterations; ++iteration)
    {
        const double p00 = riccati.at(0);
        const double p01 = riccati.at(1);
        const double p10 = riccati.at(2);
        const double p11 = riccati.at(3);

        // A = [[1, speed_dt], [0, 1]], B = [[0], [input_gain]].
        const double pb0 = p01 * input_gain;
        const double pb1 = p11 * input_gain;
        const double atpb1 = speed_dt * pb0 + pb1;
        const double denominator = std::max(
            steering_weight + input_gain * pb1, 1.0e-12);
        const double bpa0 = input_gain * p10;
        const double bpa1 = input_gain * (p10 * speed_dt + p11);

        const double apa00 = p00;
        const double apa01 = p00 * speed_dt + p01;
        const double apa10 = p10 + speed_dt * p00;
        const double apa11 = p11 + speed_dt * (p01 + p10) +
            speed_dt * speed_dt * p00;
        Matrix2 next{
            apa00 - pb0 * bpa0 / denominator + cross_track_weight,
            apa01 - pb0 * bpa1 / denominator,
            apa10 - atpb1 * bpa0 / denominator,
            apa11 - atpb1 * bpa1 / denominator + heading_weight};
        const double off_diagonal = 0.5 * (next.at(1) + next.at(2));
        next.at(1) = off_diagonal;
        next.at(2) = off_diagonal;

        double maximum_change = 0.0;
        for (std::size_t index = 0; index < next.size(); ++index)
        {
            maximum_change = std::max(
                maximum_change, std::abs(next.at(index) - riccati.at(index)));
        }
        riccati = next;
        if (maximum_change < tolerance)
        {
            break;
        }
    }
    return riccati;
}

std::array<double, 2> lqr_gain(
    const double speed_dt, const double input_gain,
    const LqgConfig& config)
{
    const double steering_weight = std::max(
        config.steering_weight, 1.0e-4);
    const Matrix2 riccati = discrete_riccati(
        speed_dt, input_gain, config.cross_track_weight,
        config.heading_weight, steering_weight);
    const double denominator = steering_weight +
        input_gain * input_gain * riccati.at(3);
    return {
        input_gain * riccati.at(2) / denominator,
        input_gain * (riccati.at(2) * speed_dt + riccati.at(3)) /
            denominator};
}

}  // namespace

LqgController::LqgController(const LqgConfig& config)
    : config_(config)
{
    if (config_.wheelbase <= 0.0 || config_.dt <= 0.0 ||
        config_.cross_track_weight < 0.0 || config_.heading_weight < 0.0 ||
        config_.steering_weight < 0.0 || config_.authority < 0.0 ||
        config_.authority > 1.0 || config_.max_steering <= 0.0 ||
        config_.max_steering_rate <= 0.0 ||
        config_.minimum_model_speed <= 0.0)
    {
        throw std::invalid_argument("Invalid LQG controller parameters");
    }
}

SteeringCommand LqgController::compute(
    const reference_trajectory::Path& path, const VehiclePose& pose,
    const double current_speed, const SteeringCommand& base)
{
    if (path.empty())
    {
        throw std::invalid_argument("LQG controller path must not be empty");
    }
    if (path.size() < 2 || !base.has_forward_target)
    {
        previous_steering_ = base.steering;
        return base;
    }

    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < path.size(); ++index)
    {
        const double candidate = distance_squared(path.at(index).position, pose);
        if (candidate < nearest_distance)
        {
            nearest_distance = candidate;
            nearest = index;
        }
    }
    const auto heading = reference_heading(path, nearest);
    if (!heading)
    {
        previous_steering_ = base.steering;
        return base;
    }

    const auto& reference = path.at(nearest).position;
    const double dx = pose.x - reference.x;
    const double dy = pose.y - reference.y;
    const std::array<double, 2> measurement{
        -std::sin(*heading) * dx + std::cos(*heading) * dy,
        wrap_angle(pose.yaw - *heading)};

    const double velocity = std::max(current_speed, config_.minimum_model_speed);
    const double speed_dt = velocity * config_.dt;
    const double input_gain = speed_dt / config_.wheelbase;
    const double curvature = reference_curvature(path, nearest);
    const std::array<double, 2> predicted{
        estimate_.at(0) + speed_dt * estimate_.at(1),
        estimate_.at(1) + input_gain * previous_steering_ -
            speed_dt * curvature};

    // P' = A P A^T + Q, with fy-code's process-noise values.
    const double p00 = covariance_.at(0);
    const double p01 = covariance_.at(1);
    const double p10 = covariance_.at(2);
    const double p11 = covariance_.at(3);
    Matrix2 predicted_covariance{
        p00 + speed_dt * (p01 + p10) + speed_dt * speed_dt * p11 + 2.0e-3,
        p01 + speed_dt * p11,
        p10 + speed_dt * p11,
        p11 + 3.0e-3};

    // K = P' (P' + R)^-1, H = I, using fy-code's measurement noise.
    const double s00 = predicted_covariance.at(0) + 1.0e-2;
    const double s01 = predicted_covariance.at(1);
    const double s10 = predicted_covariance.at(2);
    const double s11 = predicted_covariance.at(3) + 1.5e-2;
    const double determinant = std::max(s00 * s11 - s01 * s10, 1.0e-12);
    const Matrix2 inverse_innovation{
        s11 / determinant, -s01 / determinant,
        -s10 / determinant, s00 / determinant};
    Matrix2 kalman_gain{};
    for (std::size_t row = 0; row < 2; ++row)
    {
        for (std::size_t column = 0; column < 2; ++column)
        {
            kalman_gain.at(matrix_index(row, column)) =
                predicted_covariance.at(matrix_index(row, 0)) *
                inverse_innovation.at(matrix_index(0, column)) +
                predicted_covariance.at(matrix_index(row, 1)) *
                inverse_innovation.at(matrix_index(1, column));
        }
    }
    const std::array<double, 2> innovation{
        measurement.at(0) - predicted.at(0),
        measurement.at(1) - predicted.at(1)};
    for (std::size_t row = 0; row < 2; ++row)
    {
        estimate_.at(row) = predicted.at(row) +
            kalman_gain.at(matrix_index(row, 0)) * innovation.at(0) +
            kalman_gain.at(matrix_index(row, 1)) * innovation.at(1);
    }

    // P = (I - K) P', matching the source implementation.
    Matrix2 updated_covariance{};
    for (std::size_t row = 0; row < 2; ++row)
    {
        for (std::size_t column = 0; column < 2; ++column)
        {
            updated_covariance.at(matrix_index(row, column)) =
                predicted_covariance.at(matrix_index(row, column)) -
                kalman_gain.at(matrix_index(row, 0)) *
                predicted_covariance.at(matrix_index(0, column)) -
                kalman_gain.at(matrix_index(row, 1)) *
                predicted_covariance.at(matrix_index(1, column));
        }
    }
    const double covariance_off_diagonal = 0.5 *
        (updated_covariance.at(1) + updated_covariance.at(2));
    updated_covariance.at(1) = covariance_off_diagonal;
    updated_covariance.at(2) = covariance_off_diagonal;
    covariance_ = updated_covariance;

    const auto feedback = lqr_gain(speed_dt, input_gain, config_);
    const double feedforward = std::atan(config_.wheelbase * curvature);
    const double lqg_steering = feedforward -
        feedback.at(0) * estimate_.at(0) -
        feedback.at(1) * estimate_.at(1);
    double steering = config_.authority * lqg_steering +
        (1.0 - config_.authority) * base.steering;
    steering = std::clamp(
        steering, -config_.max_steering, config_.max_steering);
    const double maximum_change = config_.max_steering_rate * config_.dt;
    steering = std::clamp(
        steering, previous_steering_ - maximum_change,
        previous_steering_ + maximum_change);
    previous_steering_ = steering;

    SteeringCommand result = base;
    result.steering = steering;
    return result;
}

void LqgController::reset()
{
    estimate_ = {0.0, 0.0};
    covariance_ = {0.1, 0.0, 0.0, 0.1};
    previous_steering_ = 0.0;
}

}  // namespace motion_control
