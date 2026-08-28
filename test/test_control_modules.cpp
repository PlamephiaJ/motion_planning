#include "motion_planning/controllers.hpp"
#include "motion_planning/reference_trajectory.hpp"
#include "motion_planning/speed_limiters.hpp"

#include "gtest/gtest.h"

#include <cmath>
#include <stdexcept>

namespace
{

reference_trajectory::Sample sample(
    const double x, const double y, const double speed)
{
    reference_trajectory::Sample result;
    result.position.x = x;
    result.position.y = y;
    result.nominal_speed = speed;
    return result;
}

}  // namespace

TEST(ReferenceTrajectory, InterpolatesPositionAndItsMatchingSpeedTogether)
{
    const reference_trajectory::ReferenceTrajectory trajectory({
        sample(0.0, 0.0, 1.0), sample(10.0, 0.0, 3.0),
        sample(10.0, 10.0, 5.0), sample(0.0, 10.0, 7.0)});

    const auto middle = trajectory.sample_at(5.0);
    const auto associated = trajectory.associate(
        {sample(5.0, 1.0, 0.0).position}, 0.0);

    EXPECT_DOUBLE_EQ(5.0, middle.position.x);
    EXPECT_DOUBLE_EQ(2.0, middle.nominal_speed);
    ASSERT_EQ(1u, associated.size());
    EXPECT_DOUBLE_EQ(2.0, associated.front().nominal_speed);
}

TEST(ReferenceTrajectory, ResamplingInterpolatesNominalSpeed)
{
    const reference_trajectory::Path path = {
        sample(0.0, 0.0, 1.0), sample(2.0, 0.0, 3.0)};

    const auto resampled = reference_trajectory::resample_path(path, 1.0);

    ASSERT_EQ(3u, resampled.size());
    EXPECT_DOUBLE_EQ(1.0, resampled.at(1).position.x);
    EXPECT_DOUBLE_EQ(2.0, resampled.at(1).nominal_speed);
}

TEST(ReferenceTrajectory, RemovesDuplicateClosingSamplesAsPositionSpeedPairs)
{
    const reference_trajectory::ReferenceTrajectory trajectory({
        sample(0.0, 0.0, 1.0), sample(1.0, 0.0, 2.0),
        sample(0.0, 0.0, 1.0), sample(0.0, 0.0, 1.0)});

    EXPECT_EQ(2u, trajectory.positions().size());
    EXPECT_DOUBLE_EQ(1.5, trajectory.sample_at(0.5).nominal_speed);
}

TEST(ReferenceTrajectory, SuppliesSpeedsToOptimalSlicesAndOffLineRrtDetours)
{
    const reference_trajectory::ReferenceTrajectory trajectory({
        sample(0.0, 0.0, 1.0), sample(10.0, 0.0, 3.0),
        sample(10.0, 10.0, 5.0), sample(0.0, 10.0, 7.0)});

    const auto optimal = trajectory.slice(5.0, 10.0);
    const auto detour = trajectory.associate({
        sample(5.0, 1.0, 0.0).position,
        sample(9.0, 2.0, 0.0).position,
        sample(9.0, 5.0, 0.0).position}, 5.0);

    ASSERT_EQ(3u, optimal.size());
    EXPECT_DOUBLE_EQ(2.0, optimal.front().nominal_speed);
    EXPECT_DOUBLE_EQ(3.0, optimal.at(1).nominal_speed);
    EXPECT_DOUBLE_EQ(4.0, optimal.back().nominal_speed);
    ASSERT_EQ(3u, detour.size());
    EXPECT_DOUBLE_EQ(2.0, detour.front().nominal_speed);
    EXPECT_DOUBLE_EQ(3.4, detour.at(1).nominal_speed);
    EXPECT_DOUBLE_EQ(4.0, detour.back().nominal_speed);
}

TEST(LegacyPurePursuitController, PreservesExistingFixedLookaheadFormula)
{
    motion_control::LegacyPurePursuitConfig config;
    config.lookahead_distance = 1.0;
    config.proportional_gain = 0.25;
    config.max_steering = 0.41;
    const motion_control::LegacyPurePursuitController controller(config);
    const reference_trajectory::Path path = {
        sample(0.0, 0.0, 2.0), sample(1.0, 0.5, 2.0),
        sample(2.0, 1.0, 2.0)};

    const auto command = controller.compute(path, {});

    // Arc-length interpolation reaches (0.8944, 0.4472), then the legacy
    // formula uses the configured 1 m denominator exactly as before.
    EXPECT_NEAR(0.22360679775, command.steering, 1.0e-9);
}

TEST(PurePursuitController, UsesFyCodeSpeedDependentLookahead)
{
    motion_control::PurePursuitConfig config;
    config.wheelbase = 0.33;
    config.lookahead = 0.25;
    config.lookahead_speed_gain = 0.05;
    config.max_lookahead = 0.40;
    config.max_steering = 0.42;
    const motion_control::PurePursuitController controller(config);
    const reference_trajectory::Path path = {
        sample(0.20, 0.02, 2.0), sample(0.30, 0.03, 2.0),
        sample(0.50, 0.05, 2.0)};

    const auto slow = controller.compute(path, {}, 0.0);
    const auto fast = controller.compute(path, {}, 4.0);

    EXPECT_DOUBLE_EQ(0.25, controller.lookahead_for_speed(0.0));
    EXPECT_DOUBLE_EQ(0.40, controller.lookahead_for_speed(4.0));
    EXPECT_EQ(1u, slow.target_index);
    EXPECT_EQ(2u, fast.target_index);
    EXPECT_NEAR(
        std::atan2(2.0 * 0.33 * 0.03, 0.30 * 0.30 + 0.03 * 0.03),
        slow.steering, 1.0e-12);
}

TEST(PurePursuitController, ReportsNoForwardTargetForSafeComposition)
{
    const motion_control::PurePursuitController controller;
    const reference_trajectory::Path behind = {
        sample(-0.5, 0.0, 2.0), sample(-1.0, 0.0, 2.0)};

    const auto command = controller.compute(behind, {}, 1.0);

    EXPECT_FALSE(command.has_forward_target);
    EXPECT_DOUBLE_EQ(0.0, command.steering);
}

TEST(SteeringBandSpeedController, PreservesExistingThreeBands)
{
    constexpr double pi = 3.14159265358979323846;
    motion_control::SteeringBandSpeedConfig config;
    config.straight_speed = 2.0;
    config.medium_turn_speed = 1.0;
    config.sharp_turn_speed = 0.5;
    const motion_control::SteeringBandSpeedController controller(config);

    EXPECT_DOUBLE_EQ(2.0, controller.compute(5.0 * pi / 180.0));
    EXPECT_DOUBLE_EQ(1.0, controller.compute(15.0 * pi / 180.0));
    EXPECT_DOUBLE_EQ(0.5, controller.compute(25.0 * pi / 180.0));
}

TEST(TrajectorySpeedController, AppliesNominalPreviewBrakingEnvelope)
{
    motion_control::TrajectorySpeedConfig config;
    config.max_speed = 10.0;
    config.max_deceleration = 0.5;
    config.speed_preview_distance = 2.5;
    config.cross_track_error_gain = 0.0;
    const motion_control::TrajectorySpeedController controller(config);
    const reference_trajectory::Path path = {
        sample(0.10, 0.0, 4.0), sample(1.0, 0.0, 1.0),
        sample(3.0, 0.0, 4.0)};

    const double speed = controller.compute(path, {}, 0.0);

    EXPECT_NEAR(std::sqrt(2.0), speed, 1.0e-12);
}

TEST(TrajectorySpeedController, IsIndependentFromSteeringControllerType)
{
    motion_control::TrajectorySpeedConfig config;
    config.max_speed = 5.0;
    config.cross_track_error_gain = 0.0;
    config.max_steering = 0.4;
    config.minimum_speed_scale = 0.35;
    const motion_control::TrajectorySpeedController controller(config);
    const reference_trajectory::Path path = {
        sample(0.1, 0.0, 4.0), sample(1.0, 0.0, 4.0)};

    EXPECT_DOUBLE_EQ(4.0, controller.compute(path, {}, 0.0));
    EXPECT_DOUBLE_EQ(1.8, controller.compute(path, {}, 0.4));
}

TEST(CurvatureSpeedLimiter, UsesActualPathLocalCurvature)
{
    constexpr double half_sqrt_two = 0.70710678118654752440;
    motion_control::CurvatureSpeedLimiterConfig config;
    config.max_lateral_acceleration = 5.0;
    const motion_control::CurvatureSpeedLimiter limiter(config);
    const reference_trajectory::Path quarter_circle = {
        sample(1.0, 0.0, 10.0),
        sample(half_sqrt_two, half_sqrt_two, 10.0),
        sample(0.0, 1.0, 10.0)};

    const auto curvature =
        motion_control::CurvatureSpeedLimiter::curvatures(quarter_circle);

    ASSERT_EQ(3u, curvature.size());
    EXPECT_NEAR(1.0, curvature.at(1), 1.0e-12);
    EXPECT_NEAR(std::sqrt(5.0), limiter.compute(quarter_circle, 1), 1.0e-12);
}

TEST(BlockedPathSpeedLimiter, PreservesExistingSafetyCap)
{
    motion_control::BlockedPathSpeedLimiterConfig config;
    config.stop_distance = 0.5;
    config.gain = 2.0;
    const motion_control::BlockedPathSpeedLimiter limiter(config);

    EXPECT_DOUBLE_EQ(0.0, limiter.compute(0.4));
    EXPECT_DOUBLE_EQ(0.0, limiter.compute(0.5));
    EXPECT_DOUBLE_EQ(2.0, limiter.compute(1.5));
    EXPECT_THROW(limiter.compute(-0.1), std::invalid_argument);
}
