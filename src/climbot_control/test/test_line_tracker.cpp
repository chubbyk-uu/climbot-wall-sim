// Copyright 2026 jerry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "climbot_control/line_tracker.hpp"
#include "climbot_control/command_watchdog.hpp"
#include "climbot_control/coverage_execution.hpp"
#include "climbot_control/turn_profile.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include "gtest/gtest.h"
using namespace climbot_control;

namespace
{
climbot_interfaces::msg::CoverageTask validTask()
{
  using Task = climbot_interfaces::msg::CoverageTask;
  Task task;
  task.header.frame_id = "odom";
  task.task_id = "test-task";
  task.revision = 1U;
  task.sweep_direction = Task::SWEEP_HORIZONTAL;
  task.detection_width = 0.5;
  task.detection_length = 0.4;
  for (const auto & coordinates :
    {std::pair{-1.0F, -1.0F}, std::pair{2.0F, -1.0F},
      std::pair{2.0F, 2.0F}, std::pair{-1.0F, 2.0F}})
  {
    geometry_msgs::msg::Point32 point;
    point.x = coordinates.first;
    point.y = coordinates.second;
    task.coverage_region.points.push_back(point);
    task.motion_region.points.push_back(point);
  }
  geometry_msgs::msg::Pose first;
  first.orientation.w = 1.0;
  geometry_msgs::msg::Pose second = first;
  second.position.x = 1.0;
  task.waypoints = {first, second};
  task.segment_types = {Task::SEGMENT_SCAN};
  return task;
}
}  // namespace

TEST(LineTracker, CorrectsDownwardCrossTrackWithUpwardHeading)
{
  const auto command = trackLine({0, 0}, {1, 0}, {0, -0.1, 0}, .15, 1, 2, {});
  EXPECT_GT(command.angular, 0);
  EXPECT_NEAR(command.cross, -.1, 1e-9);
}

TEST(LineTracker, GravityFeedforwardRaisesHorizontalMotionButNotVerticalMotion)
{
  Limits limits;
  limits.gravity_slip_ratio = 0.1056;
  const auto horizontal = trackLine({0, 0}, {1, 0}, {0, 0, 0}, .15, 1, 2, limits);
  const auto vertical = trackLine({0, 0}, {0, 1}, {0, 0, std::acos(-1.0) / 2.0}, .15, 1, 2, limits);
  EXPECT_GT(horizontal.angular, 0.0);
  EXPECT_NEAR(vertical.angular, 0.0, 1e-9);
}

TEST(LineTracker, CombinesSeparateFeedforwardAndFeedbackBudgets)
{
  Limits limits;
  limits.gravity_slip_ratio = 0.1056;
  const auto command = trackLine(
    {0, 0}, {1, 0}, {0, -0.05, 0}, .15, 1, 2, limits);
  EXPECT_NEAR(command.gravity_feedforward, std::atan(0.1056), 1e-9);
  EXPECT_NEAR(command.cross_feedback, 0.05, 1e-9);
  EXPECT_NEAR(
    command.heading_correction, std::atan(0.1056) + 0.05, 1e-9);
  EXPECT_FALSE(command.correction_saturated);
}

TEST(LineTracker, IntegralCorrectsResidualWithoutBypassingTotalLimit)
{
  Limits limits;
  limits.gravity_slip_ratio = 0.1056;
  const auto residual = trackLine(
    {0, 0}, {1, 0}, {0, -0.01, 0}, .15, 1, 2, limits, 0.3, -0.1);
  EXPECT_NEAR(residual.cross_feedback, 0.04, 1e-9);
  const auto saturated = trackLine(
    {0, 0}, {1, 0}, {0, -1.0, 0}, .15, 1, 2, limits, 0.3, -1.0);
  EXPECT_NEAR(saturated.heading_correction, limits.max_heading_correction, 1e-9);
  EXPECT_TRUE(saturated.correction_saturated);
}

TEST(LineTracker, StopsForwardMotionUntilHeadingIsAligned)
{
  const auto command = trackLine({0, 0}, {1, 0}, {0, 0, .3}, .15, 1, 2, {});
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
}

TEST(LineTracker, ReducesSpeedForLargeCrossTrackErrorWithoutStopping)
{
  Limits limits;
  const auto nominal = trackLine({0, 0}, {1, 0}, {0, 0, 0}, .20, 1, 2, limits);
  const auto moderate = trackLine({0, 0}, {1, 0}, {0, .055, 0}, .20, 1, 2, limits);
  const auto large = trackLine({0, 0}, {1, 0}, {0, .20, 0}, .20, 1, 2, limits);
  EXPECT_LT(moderate.linear, nominal.linear);
  EXPECT_GT(moderate.linear, 0.0);
  EXPECT_NEAR(large.linear, nominal.linear * limits.cross_slowdown_min_scale, 1e-12);
}

TEST(LineTracker, WorksForVerticalAndDiagonalLines)
{
  EXPECT_NEAR(trackLine({0, 0}, {0, 1}, {.05, .4, 1.57079632679}, .15, 1, 2, {}).cross,
    -.05, 1e-9);
  EXPECT_GT(trackLine({0, 0}, {1, 1}, {.5, .5, 0}, .15, 1, 2, {}).angular, 0);
}

TEST(LineTracker, ArcEntrySteersMonotonicallyTowardTheNominalLine)
{
  Pose2 pose{0.0, 0.08, 0.0};
  double previous_cross = pose.y;
  for (int step = 0; step < 400 && std::abs(pose.y) > 0.005; ++step) {
    const auto command = followArcEntry(
      {0.0, 0.0}, {2.0, 0.0}, pose, 0.06, 0.20, 2.0,
      std::acos(-1.0) / 9.0, 0.25, 0.0);
    EXPECT_LT(command.heading_correction, 0.0);
    pose.yaw += command.angular * 0.02;
    pose.x += command.linear * std::cos(pose.yaw) * 0.02;
    pose.y += command.linear * std::sin(pose.yaw) * 0.02;
    EXPECT_LE(pose.y, previous_cross + 1e-9);
    EXPECT_GT(pose.y, 0.0);
    previous_cross = pose.y;
  }
  EXPECT_LT(pose.y, 0.02);
}

TEST(LineTracker, ArcEntryCommandsNoTurnOnTheNominalLine)
{
  const auto command = followArcEntry(
    {0.0, 0.0}, {0.0, -1.0}, {0.0, -0.2, -std::acos(-1.0) / 2.0},
    0.06, 0.20, 2.0, std::acos(-1.0) / 9.0, 0.25, 0.0);
  EXPECT_NEAR(command.cross, 0.0, 1e-12);
  EXPECT_NEAR(command.angular, 0.0, 1e-12);
}

TEST(LineTracker, ArcEntryApproachesFromEitherSideWithoutReversing)
{
  for (const double initial_cross : {-0.08, 0.08}) {
    Pose2 pose{0.0, initial_cross, 0.0};
    double previous_magnitude = std::abs(initial_cross);
    for (int step = 0; step < 400 && std::abs(pose.y) > 0.005; ++step) {
      const auto command = followArcEntry(
        {0.0, 0.0}, {2.0, 0.0}, pose, 0.06, 0.20, 2.0,
        std::acos(-1.0) / 9.0, 0.25, 0.0);
      EXPECT_GT(command.linear, 0.0);
      pose.yaw += command.angular * 0.02;
      pose.x += command.linear * std::cos(pose.yaw) * 0.02;
      pose.y += command.linear * std::sin(pose.yaw) * 0.02;
      EXPECT_LE(std::abs(pose.y), previous_magnitude + 1e-9);
      EXPECT_GT(pose.y * initial_cross, 0.0);
      previous_magnitude = std::abs(pose.y);
    }
    EXPECT_LT(std::abs(pose.y), 0.02);
  }
}

TEST(LineTracker, ArcEntryGravityFeedforwardRemovesWallSlipBias)
{
  constexpr double slip_ratio = 0.1056;
  constexpr double step_duration = 0.02;
  Pose2 pose{0.0, -0.11, 0.0};
  const double gravity_feedforward = std::atan(slip_ratio);
  for (int step = 0; step < 1000 && std::abs(pose.y) > 0.005; ++step) {
    const auto command = followArcEntry(
      {0.0, 0.0}, {4.0, 0.0}, pose, 0.08, 0.20, 2.0,
      std::acos(-1.0) / 9.0, 0.25, gravity_feedforward);
    pose.yaw += command.angular * step_duration;
    pose.x += command.linear * std::cos(pose.yaw) * step_duration;
    pose.y += command.linear *
      (std::sin(pose.yaw) - slip_ratio) * step_duration;
  }
  EXPECT_LT(std::abs(pose.y), 0.01);
}

TEST(LineTracker, JointWheelSaturationPreservesCurvature)
{
  Command desired{.3, 1.2};
  const auto command = rateLimit(desired, {}, 1, 1, 1, 2, .43, .3, 2);
  EXPECT_LE(std::abs(command.linear + command.angular * .43 / 2), .3 + 1e-9);
  EXPECT_NEAR(command.angular / command.linear, desired.angular / desired.linear, 1e-9);
}

TEST(LineTracker, JointWheelAccelerationIsLimited)
{
  Command desired{.3, 1.2};
  const auto command = rateLimit(desired, {}, .1, 2, 2, 20, .43, 2, .4);
  const double left = command.linear - command.angular * .43 / 2;
  const double right = command.linear + command.angular * .43 / 2;
  EXPECT_LE(std::abs(left), .04 + 1e-9);
  EXPECT_LE(std::abs(right), .04 + 1e-9);
}

// The cycle in which the wheel speed limit first binds is the one that can
// violate the wheel acceleration limit, because the speed clamp moves both
// wheels again after the acceleration clamp has finished. Here the left wheel
// is asked for -0.205 m/s of change against a 0.10 m/s budget: the
// acceleration clamp brings it to -0.091, and a speed clamp applied afterwards
// scales it back out to -0.149, half again over the limit.
TEST(LineTracker, WheelSpeedSaturationDoesNotBreakTheWheelAccelerationLimit)
{
  constexpr double kSeparation = .43;
  constexpr double kSpeedLimit = .45;
  constexpr double kAccelerationLimit = 1.;
  constexpr double kDt = .1;
  const Command previous{.44, 0.};
  // Large linear and angular limits, so the only clamps in play are the two
  // wheel-level ones whose order is under test.
  const auto command = rateLimit(
    {.45, 1.}, previous, kDt, 10., 10., 100., kSeparation, kSpeedLimit,
    kAccelerationLimit);
  const double previous_left = previous.linear - previous.angular * kSeparation / 2.;
  const double previous_right = previous.linear + previous.angular * kSeparation / 2.;
  const double left = command.linear - command.angular * kSeparation / 2.;
  const double right = command.linear + command.angular * kSeparation / 2.;
  EXPECT_LE(std::abs(left), kSpeedLimit + 1e-9);
  EXPECT_LE(std::abs(right), kSpeedLimit + 1e-9);
  EXPECT_LE(std::abs(left - previous_left), kAccelerationLimit * kDt + 1e-9);
  EXPECT_LE(std::abs(right - previous_right), kAccelerationLimit * kDt + 1e-9);
}

TEST(LineTracker, PeakWheelSpeedIsWhereRateLimitStartsTakingTheCorrectionDown)
{
  constexpr double kSeparation = .43;
  constexpr double kLinear = .35;
  constexpr double kAngular = .35;
  constexpr double kDt = .1;
  const double peak = peakWheelSpeed(kLinear, kAngular, kSeparation);
  EXPECT_NEAR(peak, kLinear + kAngular * kSeparation / 2., 1e-12);
  EXPECT_THROW(peakWheelSpeed(kLinear, kAngular, 0.), std::invalid_argument);

  // What the startup check is buying, stated against rateLimit itself: a wheel
  // limit at the peak passes the fastest command through untouched, and a
  // limit below it takes the angular correction down along with the linear
  // speed, quietly. Generous linear, angular, and acceleration limits, so the
  // wheel-speed clamp is the only one that can move anything.
  const Command previous{kLinear, kAngular};
  const auto inside = rateLimit(
    {kLinear, kAngular}, previous, kDt, 10., 10., 100., kSeparation, peak, 100.);
  EXPECT_NEAR(inside.linear, kLinear, 1e-9);
  EXPECT_NEAR(inside.angular, kAngular, 1e-9);
  const auto outside = rateLimit(
    {kLinear, kAngular}, previous, kDt, 10., 10., 100., kSeparation, peak - .01, 100.);
  EXPECT_LT(outside.linear, kLinear - 1e-9);
  EXPECT_LT(outside.angular, kAngular - 1e-9);
}

TEST(LineTracker, ExtractsYawFromGeneralNormalizedQuaternion)
{
  const double roll = 0.2, pitch = -0.3, yaw = 0.7;
  const double cr = std::cos(roll / 2.0), sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0), sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0), sy = std::sin(yaw / 2.0);
  const double x = sr * cp * cy - cr * sp * sy;
  const double y = cr * sp * cy + sr * cp * sy;
  const double z = cr * cp * sy - sr * sp * cy;
  const double w = cr * cp * cy + sr * sp * sy;
  const auto extracted = yawFromQuaternion(2.0 * x, 2.0 * y, 2.0 * z, 2.0 * w);
  ASSERT_TRUE(extracted.has_value());
  EXPECT_NEAR(*extracted, yaw, 1e-12);
  EXPECT_FALSE(yawFromQuaternion(0.0, 0.0, 0.0, 0.0).has_value());
  EXPECT_FALSE(yawFromQuaternion(
      0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 1.0).has_value());
}

TEST(TurnProfile, SelectsTriangleOrTrapezoid)
{
  const auto small = planTurn(10.0 * std::acos(-1.0) / 180.0, 0.6, 1.0);
  const auto large = planTurn(std::acos(-1.0), 0.6, 1.0);
  EXPECT_FALSE(small.isTrapezoidal());
  EXPECT_LT(small.peak_rate, 0.6);
  EXPECT_TRUE(large.isTrapezoidal());
  EXPECT_DOUBLE_EQ(large.peak_rate, 0.6);
}

TEST(TurnProfile, FinishesAtRequestedAngleAndMirrorsNegativeTurns)
{
  for (const double angle : {0.0, 0.1, 0.5, 1.5707963267948966, 3.141592653589793}) {
    const auto positive = planTurn(angle, 0.6, 1.0);
    const auto negative = planTurn(-angle, 0.6, 1.0);
    const auto positive_end = sampleTurn(positive, positive.duration);
    const auto negative_end = sampleTurn(negative, negative.duration);
    EXPECT_NEAR(positive_end.angle, angle, 1e-12);
    EXPECT_NEAR(negative_end.angle, -angle, 1e-12);
    EXPECT_DOUBLE_EQ(positive_end.angular_rate, 0.0);
    EXPECT_DOUBLE_EQ(negative_end.angular_rate, 0.0);
    EXPECT_NEAR(positive.duration, negative.duration, 1e-12);
  }
}

TEST(TurnProfile, RespectsRateAndAccelerationLimits)
{
  const auto profile = planTurn(2.4, 0.6, 1.0);
  double previous_rate = 0.0;
  constexpr double step = 0.001;
  for (double elapsed = 0.0; elapsed <= profile.duration; elapsed += step) {
    const double rate = sampleTurn(profile, elapsed).angular_rate;
    EXPECT_LE(std::abs(rate), 0.6 + 1e-12);
    EXPECT_LE(std::abs(rate - previous_rate), 1.0 * step + 1e-12);
    previous_rate = rate;
  }
}

TEST(TurnProfile, RejectsInvalidLimits)
{
  EXPECT_THROW(planTurn(1.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(planTurn(1.0, 0.6, -1.0), std::invalid_argument);
  EXPECT_THROW(
    sampleTurn(planTurn(1.0, 0.6, 1.0), std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(CoverageExecution, ValidatesImmutableTaskStructureAndBounds)
{
  auto task = validTask();
  EXPECT_FALSE(validateCoverageTask(task, "odom").has_value());
  task.revision = 0U;
  EXPECT_TRUE(validateCoverageTask(task, "odom").has_value());
  task = validTask();
  task.waypoints.back().position.x = 3.0;
  EXPECT_TRUE(validateCoverageTask(task, "odom").has_value());
  task = validTask();
  task.segment_types.clear();
  EXPECT_TRUE(validateCoverageTask(task, "odom").has_value());
}

TEST(CoverageExecution, IncludesPolygonBoundaryButRejectsOutsidePoint)
{
  const auto polygon = validTask().motion_region;
  EXPECT_TRUE(pointInPolygon(-1.0, 0.0, polygon));
  EXPECT_TRUE(pointInPolygon(0.0, 0.0, polygon));
  EXPECT_FALSE(pointInPolygon(2.1, 0.0, polygon));
}

TEST(CoverageExecution, InterpretsPolygonToleranceAsMetres)
{
  const auto polygon = validTask().motion_region;
  EXPECT_TRUE(pointInPolygon(2.015, 0.0, polygon, 0.020));
  EXPECT_FALSE(pointInPolygon(2.021, 0.0, polygon, 0.020));
}

TEST(CoverageExecution, RejectsParallelScanThatWouldRunBackward)
{
  const auto valid = parallelScanSegment({0.0, 0.0}, {1.0, 0.0}, 0.02, 0.80, 0.10);
  ASSERT_TRUE(valid.has_value());
  EXPECT_NEAR(valid->start.x, 0.80, 1e-12);
  EXPECT_NEAR(valid->start.y, 0.02, 1e-12);
  EXPECT_NEAR(valid->end.x, 1.0, 1e-12);
  EXPECT_NEAR(valid->end.y, 0.02, 1e-12);
  EXPECT_FALSE(parallelScanSegment({0.0, 0.0}, {1.0, 0.0}, 0.02, 0.91, 0.10).has_value());
  EXPECT_FALSE(parallelScanSegment({0.0, 0.0}, {1.0, 0.0}, 0.02, 1.05, 0.10).has_value());
}

TEST(CoverageExecution, DetectsSustainedCrossTrackReversalsNotSensorNoise)
{
  // Repeated zero crossings inside a small error envelope are acceptable.
  CrossTrackOscillationMonitor monitor(0.020, 0.10, 3U);
  EXPECT_FALSE(monitor.update(0.006, 0.0));
  EXPECT_FALSE(monitor.update(-0.006, 0.11));
  EXPECT_FALSE(monitor.update(0.008, 0.22));
  EXPECT_FALSE(monitor.update(-0.009, 0.33));
  EXPECT_EQ(monitor.reversalCount(), 0U);

  EXPECT_FALSE(monitor.update(0.030, 0.44));
  EXPECT_FALSE(monitor.update(-0.030, 0.55));
  EXPECT_FALSE(monitor.update(0.030, 0.66));
  EXPECT_FALSE(monitor.update(-0.030, 0.77));
  EXPECT_TRUE(monitor.update(0.030, 0.88));
  EXPECT_EQ(monitor.reversalCount(), 4U);
  monitor.reset();
  EXPECT_FALSE(monitor.update(0.004, 0.50));
  EXPECT_EQ(monitor.reversalCount(), 0U);
}

TEST(CoverageExecution, DoesNotClassifyInvalidSamplesAsOscillation)
{
  CrossTrackOscillationMonitor monitor(0.020, 0.10, 3U);
  EXPECT_FALSE(monitor.update(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_FALSE(monitor.update(0.0, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(monitor.reversalCount(), 0U);
}

namespace
{
Limits slipLimits(double ratio)
{
  Limits limits;
  limits.gravity_slip_ratio = ratio;
  limits.gravity_direction = {0.0, -1.0};
  return limits;
}

double heldYaw(double line_yaw, const Limits & limits)
{
  const double gravity_normal =
    limits.gravity_direction.x * -std::sin(line_yaw) +
    limits.gravity_direction.y * std::cos(line_yaw);
  return line_yaw + std::clamp(
    -std::atan(limits.gravity_slip_ratio * gravity_normal),
    -limits.max_gravity_feedforward, limits.max_gravity_feedforward);
}

// The reservation is defined by a fixed point: the lift must equal the drop of
// the turn the robot actually performs at the end of the line it actually
// drives. Checking that property beats pinning magic numbers, because it stays
// meaningful when the geometry or the coefficient changes.
void expectSelfConsistentReserve(
  const climbot_interfaces::msg::CoverageTask & task, std::size_t index,
  const Point2 & actual_start, double slip_per_degree, const Limits & limits)
{
  const auto segment = dynamicTransitionSegment(
    task, index, actual_start, slip_per_degree, limits);
  const auto & nominal_end = task.waypoints[index + 1U].position;
  const auto & next_end = task.waypoints[index + 2U].position;
  const double lift = segment.end.y - nominal_end.y;
  const double driven = std::atan2(
    segment.end.y - actual_start.y, segment.end.x - actual_start.x);
  const double next_line = std::atan2(
    next_end.y - nominal_end.y, next_end.x - nominal_end.x);
  const double difference = heldYaw(next_line, limits) - heldYaw(driven, limits);
  const double turn_degrees = std::abs(
    std::atan2(std::sin(difference), std::cos(difference))) * 180.0 / std::acos(-1.0);
  EXPECT_NEAR(lift, slip_per_degree * turn_degrees, 1e-4);
}
}  // namespace

TEST(CoverageExecution, HorizontalTransitionPreloadsTheSecondTurnDrop)
{
  auto task = validTask();
  using Task = climbot_interfaces::msg::CoverageTask;
  geometry_msgs::msg::Pose third = task.waypoints.back();
  third.position.y = 0.20;
  geometry_msgs::msg::Pose fourth = third;
  fourth.position.x = 0.0;
  task.waypoints.push_back(third);
  task.waypoints.push_back(fourth);
  task.segment_types = {
    Task::SEGMENT_SCAN, Task::SEGMENT_TRANSITION, Task::SEGMENT_SCAN};

  const auto limits = slipLimits(0.1056);
  const auto segment = dynamicTransitionSegment(
    task, 1U, {1.0, -0.045}, 0.0005, limits);
  EXPECT_DOUBLE_EQ(segment.start.x, 1.0);
  EXPECT_DOUBLE_EQ(segment.start.y, -0.045);
  EXPECT_NEAR(segment.end.x, 1.0, 1e-12);
  // The next line is horizontal, so the robot ends the turn holding 6 degrees
  // of up-slope: it turns 84, not the nominal 90.
  EXPECT_NEAR(segment.end.y, 0.241986, 1e-6);
  expectSelfConsistentReserve(task, 1U, {1.0, -0.045}, 0.0005, limits);
}

TEST(CoverageExecution, VerticalTransitionCapsTheTurnReserveAtTheMotionBoundary)
{
  // The reserve normally prevents a downward column being shortened by turn
  // slip. At the top edge it points outside the declared safe region, so the
  // controller must cap it rather than command an unsafe endpoint or reject a
  // task whose nominal waypoints are valid.
  auto task = validTask();
  using Task = climbot_interfaces::msg::CoverageTask;
  task.sweep_direction = Task::SWEEP_VERTICAL;
  task.motion_region.points[2].y = 1.0F;
  task.motion_region.points[3].y = 1.0F;
  task.waypoints[1].position.x = 0.0;
  task.waypoints[1].position.y = 1.0;
  geometry_msgs::msg::Pose third = task.waypoints.back();
  third.position.x = 0.20;
  geometry_msgs::msg::Pose fourth = third;
  fourth.position.y = 0.0;
  task.waypoints.push_back(third);
  task.waypoints.push_back(fourth);
  task.segment_types = {
    Task::SEGMENT_SCAN, Task::SEGMENT_TRANSITION, Task::SEGMENT_SCAN};

  const auto limits = slipLimits(0.1056);
  const auto segment = dynamicTransitionSegment(
    task, 1U, {0.0, 0.955}, 0.0005, limits);
  EXPECT_DOUBLE_EQ(segment.start.y, 0.955);
  EXPECT_LE(segment.end.y, 1.0 + 1e-6);
  EXPECT_GE(segment.end.y, 1.0 - 1e-6);
}

TEST(CoverageExecution, ReserveUsesTheDrivenHeadingNotTheNominalOne)
{
  // Starting a drop below the nominal start tilts the line the robot actually
  // drives, and lifting its end tilts it further. Reserving from the nominal
  // heading under-reserves; both cases below turn well past the nominal 90.
  auto task = validTask();
  using Task = climbot_interfaces::msg::CoverageTask;
  task.sweep_direction = Task::SWEEP_VERTICAL;
  task.waypoints[1].position.x = 0.0;
  task.waypoints[1].position.y = 1.0;
  geometry_msgs::msg::Pose third = task.waypoints.back();
  third.position.x = 0.20;
  geometry_msgs::msg::Pose fourth = third;
  fourth.position.y = 0.0;
  task.waypoints.push_back(third);
  task.waypoints.push_back(fourth);
  task.segment_types = {
    Task::SEGMENT_SCAN, Task::SEGMENT_TRANSITION, Task::SEGMENT_SCAN};

  const auto limits = slipLimits(0.0);
  const auto on_line = dynamicTransitionSegment(
    task, 1U, {0.0, 1.0}, 0.0005, limits);
  const auto dropped = dynamicTransitionSegment(
    task, 1U, {0.0, 0.955}, 0.0005, limits);
  // Same nominal geometry, different actual start. Reserving from the nominal
  // heading would give these two the same lift; the driven heading does not.
  EXPECT_GT(dropped.end.y - 1.0, on_line.end.y - 1.0 + 0.005);
  // Both exceed the nominal 90 degree turn, because lifting the end tilts the
  // driven line upward even when the robot starts exactly on the line.
  EXPECT_GT(on_line.end.y - 1.0, 0.0005 * 90.0);
  expectSelfConsistentReserve(task, 1U, {0.0, 1.0}, 0.0005, limits);
  expectSelfConsistentReserve(task, 1U, {0.0, 0.955}, 0.0005, limits);
}

TEST(CoverageExecution, ReserveFollowsTheTurnAngleNotThePreviousTurn)
{
  // The trapezoid case that made the old observed-drop floor wrong: the robot
  // has just swung 166 degrees onto a slanted transition, then only needs 14
  // degrees onto the last column. Reserving the previous turn's drop would
  // lift the end by 83 mm instead of 7 mm.
  auto task = validTask();
  using Task = climbot_interfaces::msg::CoverageTask;
  task.sweep_direction = Task::SWEEP_VERTICAL;
  task.motion_region.points[1].x = 4.0F;
  task.motion_region.points[2].x = 4.0F;
  task.motion_region.points[2].y = 5.0F;
  task.motion_region.points[3].y = 5.0F;
  task.waypoints[0].position.x = 2.761;
  task.waypoints[0].position.y = 1.395;
  task.waypoints[1].position.x = 2.761;
  task.waypoints[1].position.y = 3.961;
  geometry_msgs::msg::Pose third;
  third.position.x = 3.150;
  third.position.y = 2.405;
  geometry_msgs::msg::Pose fourth;
  fourth.position.x = 3.150;
  fourth.position.y = 1.395;
  task.waypoints.push_back(third);
  task.waypoints.push_back(fourth);
  task.segment_types = {
    Task::SEGMENT_SCAN, Task::SEGMENT_TRANSITION, Task::SEGMENT_SCAN};

  const auto limits = slipLimits(0.1056);
  const auto segment = dynamicTransitionSegment(
    task, 1U, {2.761, 3.878}, 0.0005, limits);
  EXPECT_LT(segment.end.y - third.position.y, 0.020);
  expectSelfConsistentReserve(task, 1U, {2.761, 3.878}, 0.0005, limits);
}

TEST(CoverageExecution, ShortTransitionsDoNotAmplifyIntoAWildReserve)
{
  // Below a few centimetres the driven heading is mostly position noise, and
  // the fixed point stops contracting. The reserve must stay bounded.
  auto task = validTask();
  using Task = climbot_interfaces::msg::CoverageTask;
  task.waypoints[1].position.x = 0.0;
  task.waypoints[1].position.y = 0.0;
  geometry_msgs::msg::Pose third;
  third.position.x = 0.005;
  third.position.y = 0.0;
  geometry_msgs::msg::Pose fourth;
  fourth.position.x = 0.005;
  fourth.position.y = -1.0;
  task.waypoints.push_back(third);
  task.waypoints.push_back(fourth);
  task.segment_types = {
    Task::SEGMENT_SCAN, Task::SEGMENT_TRANSITION, Task::SEGMENT_SCAN};

  const auto segment = dynamicTransitionSegment(
    task, 1U, {0.0, -0.001}, 0.0005, slipLimits(0.1056));
  EXPECT_GE(segment.end.y - third.position.y, 0.0);
  EXPECT_LE(segment.end.y - third.position.y, 0.0005 * 180.0);
}

TEST(CoverageExecution, VerticalTaskStillPreloadsATopEdgeFinishingScan)
{
  // The last column ran downward, so the return leg retraces it straight back
  // up before the finishing line heads off across the top.
  auto task = validTask();
  using Task = climbot_interfaces::msg::CoverageTask;
  task.sweep_direction = Task::SWEEP_VERTICAL;
  task.waypoints[0].position.x = 0.0;
  task.waypoints[0].position.y = 1.0;
  task.waypoints[1].position.x = 0.0;
  task.waypoints[1].position.y = 0.0;
  geometry_msgs::msg::Pose entry;
  entry.position.x = 0.0;
  entry.position.y = 1.0;
  geometry_msgs::msg::Pose finish = entry;
  finish.position.x = -1.0;
  task.waypoints.push_back(entry);
  task.waypoints.push_back(finish);
  task.segment_types = {
    Task::SEGMENT_SCAN, Task::SEGMENT_TRANSITION, Task::SEGMENT_SCAN};

  const auto limits = slipLimits(0.1056);
  const auto segment = dynamicTransitionSegment(
    task, 1U, {0.0, -0.045}, 0.0005, limits);
  EXPECT_NEAR(segment.end.x, 0.0, 1e-12);
  EXPECT_NEAR(segment.end.y, 1.041986, 1e-6);
  expectSelfConsistentReserve(task, 1U, {0.0, -0.045}, 0.0005, limits);
}

TEST(CommandWatchdog, StopsBeforeFirstCommandAndAfterTimeout)
{
  CommandWatchdog watchdog(.4);
  EXPECT_TRUE(watchdog.timedOut(0.0));
  watchdog.accept({.1, -.2}, 1.0);
  EXPECT_FALSE(watchdog.timedOut(1.4));
  EXPECT_NEAR(watchdog.commandAt(1.4).linear, .1, 1e-9);
  EXPECT_TRUE(watchdog.timedOut(1.401));
  EXPECT_DOUBLE_EQ(watchdog.commandAt(1.401).linear, 0.0);
  EXPECT_DOUBLE_EQ(watchdog.commandAt(.5).angular, 0.0);
}

TEST(CommandWatchdog, RejectsNonFiniteCommandsAndTimes)
{
  CommandWatchdog watchdog(.4);
  EXPECT_TRUE(watchdog.accept({.1, .2}, 1.0));
  EXPECT_FALSE(watchdog.accept(
      {std::numeric_limits<double>::infinity(), 0.0}, 1.1));
  EXPECT_TRUE(watchdog.timedOut(1.1));
  EXPECT_FALSE(watchdog.accept({.1, .2}, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(watchdog.timedOut(1.2));
}

TEST(CommandWatchdog, RejectsANonFiniteOrNonPositiveTimeout)
{
  EXPECT_THROW(CommandWatchdog(0.0), std::invalid_argument);
  EXPECT_THROW(CommandWatchdog(-0.1), std::invalid_argument);
  EXPECT_THROW(
    CommandWatchdog(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(
    CommandWatchdog(std::numeric_limits<double>::infinity()), std::invalid_argument);
}
