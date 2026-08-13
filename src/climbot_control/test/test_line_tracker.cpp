#include "climbot_control/line_tracker.hpp"
#include "climbot_control/command_watchdog.hpp"
#include <cmath>
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
