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

#include <gtest/gtest.h>

#include <cstdint>

#include "climbot_gazebo/clock_throttle.hpp"

namespace
{

/// Drive a throttle with a fixed input step for a stretch of simulation time.
void feed(
  climbot_gazebo::ClockThrottle & throttle, int64_t step_ns, int64_t duration_ns,
  int64_t start_ns = 0)
{
  for (int64_t stamp = start_ns; stamp <= start_ns + duration_ns; stamp += step_ns) {
    throttle.shouldPublish(stamp);
  }
}

constexpr int64_t kGazeboStepNs = 1'000'000;
constexpr int64_t kTwoSecondsNs = 2'000'000'000;

}  // namespace

TEST(ClockThrottle, SelectsExactFiveHundredHertzCadence)
{
  climbot_gazebo::ClockThrottle throttle(500.0);
  EXPECT_TRUE(throttle.shouldPublish(0));
  EXPECT_FALSE(throttle.shouldPublish(1'000'000));
  EXPECT_TRUE(throttle.shouldPublish(2'000'000));
  EXPECT_FALSE(throttle.shouldPublish(3'000'000));
  EXPECT_TRUE(throttle.shouldPublish(4'000'000));
}

TEST(ClockThrottle, DoesNotBurstAfterAForwardJump)
{
  climbot_gazebo::ClockThrottle throttle(200.0);
  EXPECT_TRUE(throttle.shouldPublish(0));
  EXPECT_TRUE(throttle.shouldPublish(25'000'000));
  EXPECT_FALSE(throttle.shouldPublish(26'000'000));
  EXPECT_TRUE(throttle.shouldPublish(30'000'000));
}

TEST(ClockThrottle, PublishesImmediatelyAfterTimeMovesBackward)
{
  climbot_gazebo::ClockThrottle throttle(200.0);
  EXPECT_TRUE(throttle.shouldPublish(10'000'000));
  EXPECT_FALSE(throttle.shouldPublish(11'000'000));
  EXPECT_TRUE(throttle.shouldPublish(0));
  EXPECT_FALSE(throttle.shouldPublish(1'000'000));
  EXPECT_TRUE(throttle.shouldPublish(5'000'000));
}

TEST(ClockThrottle, RejectsInvalidRates)
{
  EXPECT_THROW(climbot_gazebo::ClockThrottle(0.0), std::invalid_argument);
  EXPECT_THROW(climbot_gazebo::ClockThrottle(-1.0), std::invalid_argument);
}

TEST(ClockThrottle, DeliversRatesThatDivideTheGazeboStep)
{
  for (const double requested : {1000.0, 500.0, 250.0, 200.0, 125.0, 100.0}) {
    climbot_gazebo::ClockThrottle throttle(requested);
    feed(throttle, kGazeboStepNs, kTwoSecondsNs);
    EXPECT_NEAR(throttle.measuredOutputRateHz(), requested, 0.01) << "requested " << requested;
    EXPECT_NEAR(throttle.measuredInputRateHz(), 1000.0, 0.01);
  }
}

TEST(ClockThrottle, RoundsAnUnreachableRateDownToTheNextRungInsteadOfHittingIt)
{
  // A 1 ms input can only be divided into 1000/k Hz. 400 Hz falls between the
  // 500 and 333.3 rungs, so the stream settles on the lower one. The node
  // reports the measured rate for exactly this reason: the request alone would
  // have claimed 400 Hz.
  climbot_gazebo::ClockThrottle throttle(400.0);
  feed(throttle, kGazeboStepNs, kTwoSecondsNs);
  EXPECT_NEAR(throttle.measuredOutputRateHz(), 1000.0 / 3.0, 0.01);
  EXPECT_GT(throttle.requestedRateHz() - throttle.measuredOutputRateHz(), 60.0);
}

TEST(ClockThrottle, CountsEveryInputAndOnlyThePublishedOutputs)
{
  climbot_gazebo::ClockThrottle throttle(250.0);
  feed(throttle, kGazeboStepNs, kTwoSecondsNs);
  EXPECT_EQ(throttle.inputs(), 2001U);
  EXPECT_EQ(throttle.outputs(), 501U);
}

TEST(ClockThrottle, MeasuresNothingBeforeTwoSamplesArrive)
{
  climbot_gazebo::ClockThrottle throttle(500.0);
  EXPECT_EQ(throttle.measuredInputRateHz(), 0.0);
  EXPECT_EQ(throttle.measuredOutputRateHz(), 0.0);
  EXPECT_TRUE(throttle.shouldPublish(0));
  EXPECT_EQ(throttle.measuredOutputRateHz(), 0.0);
  EXPECT_FALSE(throttle.shouldPublish(1'000'000));
  EXPECT_NEAR(throttle.measuredInputRateHz(), 1000.0, 1e-6);
  EXPECT_EQ(throttle.measuredOutputRateHz(), 0.0);
}

TEST(ClockThrottle, RestartsTheMeasurementWindowAfterASimulatorReset)
{
  climbot_gazebo::ClockThrottle throttle(500.0);
  feed(throttle, kGazeboStepNs, kTwoSecondsNs, 10'000'000'000);
  ASSERT_NEAR(throttle.measuredOutputRateHz(), 500.0, 0.01);

  // Reset to zero. Averaging across the ten-second gap would report a rate
  // near zero, which describes neither side of the discontinuity.
  feed(throttle, kGazeboStepNs, kTwoSecondsNs);
  EXPECT_NEAR(throttle.measuredOutputRateHz(), 500.0, 0.01);
  EXPECT_NEAR(throttle.measuredInputRateHz(), 1000.0, 0.01);
  EXPECT_EQ(throttle.inputs(), 2001U);
}

TEST(GapStatistics, CountsEachThresholdCumulatively)
{
  climbot_gazebo::GapStatistics gaps;
  gaps.add(0.010);
  gaps.add(0.060);
  gaps.add(0.150);
  gaps.add(0.300);
  EXPECT_EQ(gaps.samples(), 4U);
  EXPECT_NEAR(gaps.maxS(), 0.300, 1e-9);
  EXPECT_EQ(gaps.atLeast(0), 3U);   // >= 50 ms
  EXPECT_EQ(gaps.atLeast(1), 2U);   // >= 100 ms
  EXPECT_EQ(gaps.atLeast(2), 1U);   // >= 200 ms
  EXPECT_EQ(gaps.atLeast(3), 1U);   // >= 250 ms
}

TEST(GapStatistics, StartsEmptyAndStaysBoundedInSpace)
{
  climbot_gazebo::GapStatistics gaps;
  EXPECT_EQ(gaps.samples(), 0U);
  EXPECT_EQ(gaps.maxS(), 0.0);
  for (int i = 0; i < 100000; ++i) {
    gaps.add(0.001);
  }
  EXPECT_EQ(gaps.samples(), 100000U);
  EXPECT_EQ(gaps.atLeast(0), 0U);
  EXPECT_NEAR(gaps.maxS(), 0.001, 1e-9);
}

TEST(ClockThrottle, KeepsInputAndOutputGapsApart)
{
  climbot_gazebo::ClockThrottle throttle(500.0);
  throttle.recordInputGap(0.001);
  throttle.recordInputGap(0.001);
  throttle.recordOutputGap(0.300);
  EXPECT_EQ(throttle.inputGaps().samples(), 2U);
  EXPECT_EQ(throttle.inputGaps().atLeast(3), 0U);
  EXPECT_EQ(throttle.outputGaps().samples(), 1U);
  EXPECT_EQ(throttle.outputGaps().atLeast(3), 1U);
}
