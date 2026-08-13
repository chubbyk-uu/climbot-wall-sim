#include "climbot_control/line_tracker.hpp"
#include "climbot_control/command_watchdog.hpp"
#include <cmath>
#include <limits>
#include "gtest/gtest.h"
using namespace climbot_control;
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

TEST(LineTracker, WorksForVerticalAndDiagonalLines)
{
  EXPECT_NEAR(trackLine({0, 0}, {0, 1}, {.05, .4, 1.57079632679}, .15, 1, 2, {}).cross,
    -.05, 1e-9);
  EXPECT_GT(trackLine({0, 0}, {1, 1}, {.5, .5, 0}, .15, 1, 2, {}).angular, 0);
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
