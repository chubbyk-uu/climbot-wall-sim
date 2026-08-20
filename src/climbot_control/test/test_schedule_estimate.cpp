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

// The schedule an operator plans work from, tested without running a task.
//
// These numbers - the progress bar, the countdown, the planned total - lived
// inside the tracker node, where the only way to exercise them was to drive a
// whole task in simulation. Everything below is arithmetic over a plan and two
// fractions the controller supplies, so it belongs to an object that can be
// asked a question directly.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "climbot_control/schedule_estimate.hpp"

namespace
{

/// The shipped control.yaml values, as in test_segment_duration.cpp, so these
/// expectations are comparable with the recorded baselines.
climbot_control::DurationModel model()
{
  climbot_control::DurationModel value;
  value.cruise_speed = 0.20;
  value.linear_acceleration = 0.20;
  value.braking_deceleration = 0.12;
  value.max_turn_rate = 0.35;
  value.turn_acceleration = 1.00;
  value.align_settle_s = 0.50;
  value.goal_settle_s = 0.30;
  return value;
}

/// Two equal legs at right angles, so a turn sits between them.
std::vector<climbot_control::Point2> lShape()
{
  return {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
}

}  // namespace

TEST(ScheduleEstimate, AnUnplannedEstimateReportsNothingRatherThanZeroProgress)
{
  climbot_control::ScheduleEstimate schedule;
  EXPECT_EQ(schedule.totalDuration(), 0.0);
  EXPECT_EQ(schedule.plannedTotal(), 0.0);
  EXPECT_FALSE(schedule.hasApproach());
  // No plan means no denominator, and a fraction of nothing is not 1.
  EXPECT_EQ(schedule.progress(0U, 0U, 1.0, 1.0), 0.0);
}

TEST(ScheduleEstimate, TheBarCountsSegmentsAndTheScheduleCountsTheApproachToo)
{
  // The distinction the progress bar depends on: the drive to the first
  // waypoint is real time an operator waits through, but it is not one of the
  // task's segments, so the bar must read zero through it while the schedule
  // must not.
  climbot_control::ScheduleEstimate schedule;
  schedule.plan(lShape(), {-3.0, 0.0}, 0.0, model(), 0.03);
  EXPECT_TRUE(schedule.hasApproach());
  EXPECT_GT(schedule.plannedTotal(), schedule.totalDuration());
  EXPECT_GT(schedule.totalDuration(), 0.0);
}

TEST(ScheduleEstimate, ARobotAlreadyAtTheFirstWaypointGetsNoApproach)
{
  climbot_control::ScheduleEstimate schedule;
  schedule.plan(lShape(), {0.0, 0.0}, 0.0, model(), 0.03);
  EXPECT_FALSE(schedule.hasApproach());
  EXPECT_DOUBLE_EQ(schedule.plannedTotal(), schedule.totalDuration());
  EXPECT_EQ(schedule.approachRemaining(0.0, 1.0), 0.0);
}

TEST(ScheduleEstimate, ProgressIsWeightedByDurationAndNotBySegmentCount)
{
  // A short transition and a long scan line must not advance the bar equally.
  // Weighting by count made a 0.44 m transition move it as far as a 4.5 m
  // scan, which is the measurement this weighting exists for.
  climbot_control::ScheduleEstimate schedule;
  schedule.plan({{0.0, 0.0}, {0.44, 0.0}, {0.44, 4.5}}, {0.0, 0.0}, 0.0, model(), 0.03);
  const double after_short = schedule.progress(1U, 1U, 0.0, 0.0);
  const double after_both = schedule.progress(2U, 1U, 1.0, 1.0);
  EXPECT_GT(after_both, after_short);
  EXPECT_LT(after_short, 0.5) << "the short transition took more than half the bar";
}

TEST(ScheduleEstimate, ProgressAdvancesWithinASegmentAndNeverLeavesZeroToOne)
{
  climbot_control::ScheduleEstimate schedule;
  schedule.plan(lShape(), {0.0, 0.0}, 0.0, model(), 0.03);
  const double turning = schedule.progress(0U, 0U, 0.5, 0.0);
  const double driving = schedule.progress(0U, 0U, 1.0, 0.5);
  EXPECT_GT(driving, turning);
  EXPECT_GT(turning, 0.0);
  // Fractions the controller could hand over out of range must not escape.
  EXPECT_LE(schedule.progress(2U, 1U, 5.0, 5.0), 1.0);
  EXPECT_GE(schedule.progress(0U, 0U, -5.0, -5.0), 0.0);
  // A segment index past the plan is not a completed task; it is no answer.
  EXPECT_EQ(schedule.progress(0U, 99U, 1.0, 1.0), 0.0);
}

TEST(ScheduleEstimate, TheApproachCountsDownRatherThanDroppingOnArrival)
{
  // Carrying the leg as a block that only disappears on arrival left the
  // countdown standing still and then jumping by the whole leg at once.
  climbot_control::ScheduleEstimate schedule;
  schedule.plan(lShape(), {-3.0, 0.0}, 3.14159, model(), 0.03);
  const double whole = schedule.approachRemaining(0.0, 1.0);
  const double turned = schedule.approachRemaining(1.0, 1.0);
  const double half_driven = schedule.approachRemaining(1.0, 0.5);
  const double arrived = schedule.approachRemaining(1.0, 0.0);
  EXPECT_GT(whole, turned);
  EXPECT_GT(turned, half_driven);
  EXPECT_GT(half_driven, arrived);
  EXPECT_EQ(arrived, 0.0);
}

TEST(ScheduleEstimate, WhatIsLeftFollowsTheBarAndCarriesTheLag)
{
  climbot_control::ScheduleEstimate schedule;
  schedule.plan(lShape(), {0.0, 0.0}, 0.0, model(), 0.03);
  const double total = schedule.totalDuration();
  EXPECT_NEAR(schedule.remaining(0.0, 0.0, 0.0), total, 1e-9);
  EXPECT_NEAR(schedule.remaining(0.5, 0.0, 0.0), total / 2.0, 1e-9);
  EXPECT_NEAR(schedule.remaining(1.0, 0.0, 0.0), 0.0, 1e-9);
  // Behind schedule means longer left, not a shorter bar.
  EXPECT_NEAR(schedule.remaining(0.5, 0.0, 4.0), total / 2.0 + 4.0, 1e-9);
  // Ahead of schedule can shorten it, but never past zero: a negative
  // countdown is not a thing an operator can act on.
  EXPECT_EQ(schedule.remaining(1.0, 0.0, -10.0), 0.0);
}

TEST(ScheduleEstimate, PlanningAgainReplacesTheJobRatherThanAddingToIt)
{
  climbot_control::ScheduleEstimate schedule;
  schedule.plan(lShape(), {0.0, 0.0}, 0.0, model(), 0.03);
  const double first = schedule.totalDuration();
  schedule.plan(lShape(), {0.0, 0.0}, 0.0, model(), 0.03);
  EXPECT_DOUBLE_EQ(schedule.totalDuration(), first);
  schedule.clear();
  EXPECT_EQ(schedule.totalDuration(), 0.0);
  EXPECT_EQ(schedule.plannedTotal(), 0.0);
}
