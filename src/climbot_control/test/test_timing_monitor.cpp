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

#include "climbot_control/timing_monitor.hpp"

#include "gtest/gtest.h"

namespace
{

TEST(TimingMonitor, CountsThresholdsAndReportsConservativeQuantiles)
{
  climbot_control::DurationStatistics statistics;
  statistics.add(20'000'000LL);
  statistics.add(50'000'000LL);
  statistics.add(251'000'000LL);
  EXPECT_EQ(statistics.sampleCount(), 3U);
  EXPECT_EQ(statistics.maxNs(), 251'000'000LL);
  EXPECT_EQ(statistics.thresholdCount(0U), 2U);
  EXPECT_EQ(statistics.thresholdCount(1U), 1U);
  EXPECT_EQ(statistics.thresholdCount(2U), 1U);
  EXPECT_EQ(statistics.thresholdCount(3U), 1U);
  EXPECT_EQ(statistics.quantileNs(0.999), 251'000'000LL);
  EXPECT_EQ(statistics.quantileNs(0.0), 20'000'000LL);
}

TEST(TimingMonitor, DetectsIntervalsAndClockRollbackWithoutFalseGap)
{
  climbot_control::TimingSeries series;
  const auto first = series.record(1'000LL);
  EXPECT_TRUE(first.initialized);
  const auto normal = series.record(21'000'000LL);
  EXPECT_FALSE(normal.regressed);
  EXPECT_EQ(normal.observation.duration_ns, 20'999'000LL);
  const auto gap = series.record(321'000'000LL);
  EXPECT_TRUE(gap.observation.crossed[0U]);
  EXPECT_TRUE(gap.observation.crossed[1U]);
  EXPECT_TRUE(gap.observation.crossed[2U]);
  EXPECT_TRUE(gap.observation.crossed[3U]);
  const auto rollback = series.record(200'000'000LL);
  EXPECT_TRUE(rollback.regressed);
  EXPECT_EQ(series.rollbackCount(), 1U);
  EXPECT_EQ(series.statistics().sampleCount(), 2U);
  const auto recovered = series.record(220'000'000LL);
  EXPECT_EQ(recovered.observation.duration_ns, 20'000'000LL);
}

TEST(TimingMonitor, LongMonitorSchedulingDelayIsARegularDurationSample)
{
  climbot_control::DurationStatistics lateness;
  const auto observation = lateness.add(352'000'000LL);
  EXPECT_TRUE(observation.crossed[3U]);
  EXPECT_EQ(lateness.thresholdCount(3U), 1U);
}

}  // namespace
