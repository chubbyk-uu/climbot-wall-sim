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

// When a segment counts as arrived at - the three conditions, and why each.
//
// This decided the end of every segment of every run and could only be
// exercised by driving one. Each of the three parts below has a failure it
// exists to prevent, and none of those failures is visible from a passing
// task: a single band flickers, no settle time reports a robot that slid back
// out as finished, and no speed test finishes a segment the robot drove
// straight through.

#include <gtest/gtest.h>

#include "climbot_control/segment_arrival.hpp"

namespace
{

/// The shipped control.yaml values for an ordinary segment goal.
climbot_control::SegmentArrival::Tolerances tolerances()
{
  climbot_control::SegmentArrival::Tolerances value;
  value.position = 0.03;
  value.position_exit = 0.04;
  value.heading = 0.034906585;
  value.heading_exit = 0.052359878;
  value.linear_speed = 0.01;
  value.angular_speed = 0.02;
  value.settle_s = 0.30;
  return value;
}

}  // namespace

TEST(SegmentArrival, ArrivingIsNotEnoughOnItsOwn)
{
  // The robot slips while stopped on a wall. A ball it leaves as fast as it
  // enters is not arrival, so the tight band has to hold for the settle time.
  climbot_control::SegmentArrival arrival;
  EXPECT_FALSE(arrival.update(0.0, 0.01, 0.0, 0.0, 0.0, tolerances()));
  EXPECT_TRUE(arrival.settling());
  EXPECT_FALSE(arrival.update(0.2, 0.01, 0.0, 0.0, 0.0, tolerances()));
  EXPECT_TRUE(arrival.update(0.31, 0.01, 0.0, 0.0, 0.0, tolerances()));
}

TEST(SegmentArrival, SlidingBackOutAbandonsTheSettleRatherThanFinishing)
{
  climbot_control::SegmentArrival arrival;
  EXPECT_FALSE(arrival.update(0.0, 0.01, 0.0, 0.0, 0.0, tolerances()));
  // Past the loose band: this is not drift, it is having left.
  EXPECT_FALSE(arrival.update(0.1, 0.05, 0.0, 0.0, 0.0, tolerances()));
  EXPECT_FALSE(arrival.settling());
  // The clock starts over, so the old start cannot finish the new settle.
  EXPECT_FALSE(arrival.update(0.2, 0.01, 0.0, 0.0, 0.0, tolerances()));
  EXPECT_FALSE(arrival.update(0.45, 0.01, 0.0, 0.0, 0.0, tolerances()));
  EXPECT_TRUE(arrival.update(0.55, 0.01, 0.0, 0.0, 0.0, tolerances()));
}

TEST(SegmentArrival, DriftBetweenTheTwoBandsKeepsTheSettleButDoesNotFinishIt)
{
  // The reason there are two bands. A robot settling right on the tight edge
  // would never finish if every crossing restarted the clock, and would finish
  // from outside the tight band if the loose one could complete on its own.
  climbot_control::SegmentArrival arrival;
  const auto limits = tolerances();
  EXPECT_FALSE(arrival.update(0.0, 0.01, 0.0, 0.0, 0.0, limits));
  // 35 mm: outside the 30 mm entry band, inside the 40 mm exit band.
  EXPECT_FALSE(arrival.update(0.4, 0.035, 0.0, 0.0, 0.0, limits));
  EXPECT_TRUE(arrival.settling()) << "the settle was abandoned by ordinary drift";
  // Back inside the tight band with the original settle long since elapsed.
  EXPECT_TRUE(arrival.update(0.41, 0.01, 0.0, 0.0, 0.0, limits));
}

TEST(SegmentArrival, PassingThroughTheGoalAtSpeedIsNotArrival)
{
  // Position alone is satisfied by a robot on its way past.
  climbot_control::SegmentArrival arrival;
  const auto limits = tolerances();
  EXPECT_FALSE(arrival.update(0.0, 0.0, 0.0, 0.2, 0.0, limits));
  EXPECT_FALSE(arrival.settling());
  EXPECT_FALSE(arrival.update(0.5, 0.0, 0.0, 0.2, 0.0, limits));
  // Still spinning counts as moving too.
  EXPECT_FALSE(arrival.update(1.0, 0.0, 0.0, 0.0, 0.5, limits));
  EXPECT_FALSE(arrival.settling());
}

TEST(SegmentArrival, HeadingIsJudgedBothWaysRoundZero)
{
  climbot_control::SegmentArrival arrival;
  const auto limits = tolerances();
  EXPECT_FALSE(arrival.update(0.0, 0.0, -0.02, 0.0, 0.0, limits));
  EXPECT_TRUE(arrival.settling()) << "a negative heading error was read as large";
  EXPECT_TRUE(arrival.update(0.4, 0.0, -0.02, 0.0, 0.0, limits));

  // And a heading past the exit band abandons it, whichever side it is on.
  arrival.reset();
  EXPECT_FALSE(arrival.update(1.0, 0.0, 0.0, 0.0, 0.0, limits));
  EXPECT_FALSE(arrival.update(1.1, 0.0, -0.09, 0.0, 0.0, limits));
  EXPECT_FALSE(arrival.settling());
}

TEST(SegmentArrival, AResetSegmentDoesNotInheritThePreviousOnesSettle)
{
  climbot_control::SegmentArrival arrival;
  EXPECT_FALSE(arrival.update(0.0, 0.0, 0.0, 0.0, 0.0, tolerances()));
  arrival.reset();
  EXPECT_FALSE(arrival.settling());
  // Would have been complete on the old clock; must not be on the new one.
  EXPECT_FALSE(arrival.update(0.5, 0.0, 0.0, 0.0, 0.0, tolerances()));
  EXPECT_TRUE(arrival.update(0.85, 0.0, 0.0, 0.0, 0.0, tolerances()));
}

TEST(SegmentArrival, TheLooseStartApproachBandsAreHonouredAsGiven)
{
  // The start approach turns in place and slips while stopped, so it runs on
  // 50 and 60 mm rather than 30 and 40. Nothing in here should know that; it
  // has to take whichever bands it is handed.
  climbot_control::SegmentArrival arrival;
  auto approach = tolerances();
  approach.position = 0.05;
  approach.position_exit = 0.06;
  EXPECT_FALSE(arrival.update(0.0, 0.045, 0.0, 0.0, 0.0, approach));
  EXPECT_TRUE(arrival.settling());
  EXPECT_TRUE(arrival.update(0.4, 0.045, 0.0, 0.0, 0.0, approach));
  // The same 45 mm against the ordinary bands is not arrival at all.
  arrival.reset();
  EXPECT_FALSE(arrival.update(1.0, 0.045, 0.0, 0.0, 0.0, tolerances()));
  EXPECT_FALSE(arrival.settling());
}
