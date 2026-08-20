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

// The decisions taken at the top of a scan line, asked directly.
//
// These three - where the robot is relative to the line, what to do about the
// offset it ended the turn with, and whether entering the first scan line can
// work at all - decided the shape of every run, and could only be exercised by
// driving one. They are arithmetic; they can be asked.

#include <gtest/gtest.h>

#include "climbot_control/scan_entry.hpp"

using climbot_control::ScanEntry;

namespace
{
// The shipped control.yaml thresholds.
constexpr double kParallel = 0.045;
constexpr double kMaximum = 0.12;
constexpr double kSlipPerDegree = 0.0005;
}  // namespace

TEST(ScanEntry, OffsetIsMeasuredAlongAndAcrossTheLineRatherThanInWorldAxes)
{
  // A line running up the wall: being to its right is a cross offset, being
  // above its start is along. Getting these two the wrong way round would
  // freeze parallel lines at the wrong spacing and read as a planning fault.
  const auto offset = climbot_control::offsetFromLine(
    {1.0, 0.0}, {1.0, 4.0}, {1.05, 2.0});
  EXPECT_NEAR(offset.along, 2.0, 1e-9);
  EXPECT_NEAR(offset.cross, -0.05, 1e-9);

  // Sign follows the line's own direction, so the same robot on a line running
  // the other way is offset the other way.
  const auto reversed = climbot_control::offsetFromLine(
    {1.0, 4.0}, {1.0, 0.0}, {1.05, 2.0});
  EXPECT_NEAR(reversed.along, 2.0, 1e-9);
  EXPECT_NEAR(reversed.cross, 0.05, 1e-9);
}

TEST(ScanEntry, ALineWithNoLengthGivesNoOffsetRatherThanInfinity)
{
  const auto offset = climbot_control::offsetFromLine({1.0, 1.0}, {1.0, 1.0}, {2.0, 2.0});
  EXPECT_EQ(offset.along, 0.0);
  EXPECT_EQ(offset.cross, 0.0);
}

TEST(ScanEntry, SmallOffsetsAreAcceptedAsAParallelScanLine)
{
  EXPECT_EQ(climbot_control::classifyScanOffset(0.0, kParallel, kMaximum),
    ScanEntry::LOCK_PARALLEL);
  EXPECT_EQ(climbot_control::classifyScanOffset(0.03, kParallel, kMaximum),
    ScanEntry::LOCK_PARALLEL);
  // Either side of the line, since the sign only says which way it drifted.
  EXPECT_EQ(climbot_control::classifyScanOffset(-0.03, kParallel, kMaximum),
    ScanEntry::LOCK_PARALLEL);
}

TEST(ScanEntry, MiddlingOffsetsAreDrivenOutWithOneForwardArc)
{
  EXPECT_EQ(climbot_control::classifyScanOffset(0.08, kParallel, kMaximum),
    ScanEntry::ARC_ENTRY);
  EXPECT_EQ(climbot_control::classifyScanOffset(-0.08, kParallel, kMaximum),
    ScanEntry::ARC_ENTRY);
}

TEST(ScanEntry, OffsetsBeyondRecoveryAreRefusedRatherThanAttempted)
{
  EXPECT_EQ(climbot_control::classifyScanOffset(0.13, kParallel, kMaximum),
    ScanEntry::TOO_FAR);
  EXPECT_EQ(climbot_control::classifyScanOffset(-0.13, kParallel, kMaximum),
    ScanEntry::TOO_FAR);
}

TEST(ScanEntry, TheThresholdsThemselvesAreInclusiveOnTheAcceptingSide)
{
  // Exactly at a threshold takes the gentler branch, both times: a run must
  // not turn on whether a float landed a bit either side of a configured
  // number.
  EXPECT_EQ(climbot_control::classifyScanOffset(kParallel, kParallel, kMaximum),
    ScanEntry::LOCK_PARALLEL);
  EXPECT_EQ(climbot_control::classifyScanOffset(kMaximum, kParallel, kMaximum),
    ScanEntry::ARC_ENTRY);
}

TEST(ScanEntry, AScanRunningWithGravityAbsorbsTheTurnDropAlongItsOwnLength)
{
  // Gravity is down the wall, -Y. The robot drives up at the first waypoint,
  // so its approach heading is +90 degrees, and the two scans below turn away
  // from that by different amounts.
  //
  // The scan running straight down turns further - 180 degrees against the
  // horizontal one's 90 - and therefore slides further. It still costs nothing,
  // because the slide is along its own length, where it is not an offset at
  // all. Budgeting for the worst case regardless of heading would refuse
  // vertical sweeps that work perfectly.
  const climbot_control::Point2 gravity{0.0, -1.0};
  const climbot_control::Point2 robot{2.0, -3.0};
  const climbot_control::Point2 first{2.0, 0.0};
  const climbot_control::Point2 lifted{2.0, 0.0};  // nothing reserved

  const double down_the_wall = climbot_control::firstScanEntryBudget(
    robot, first, {2.0, -4.0}, lifted, gravity, kSlipPerDegree, 0.05);
  const double across_the_wall = climbot_control::firstScanEntryBudget(
    robot, first, {6.0, 0.0}, lifted, gravity, kSlipPerDegree, 0.05);

  EXPECT_NEAR(down_the_wall, 0.05, 1e-9) << "a scan along gravity carries no offset";
  // 90 degrees at 0.5 mm per degree is 45 mm, all of it normal to this scan.
  EXPECT_NEAR(across_the_wall, 0.05 + 0.045, 1e-9);
}

TEST(ScanEntry, LiftingTheApproachTargetPaysForTheDropInAdvance)
{
  const climbot_control::Point2 gravity{0.0, -1.0};
  const climbot_control::Point2 robot{2.0, -3.0};
  const climbot_control::Point2 first{2.0, 0.0};
  const climbot_control::Point2 second{6.0, 0.0};

  const double unreserved = climbot_control::firstScanEntryBudget(
    robot, first, second, {2.0, 0.0}, gravity, kSlipPerDegree, 0.05);
  const double reserved = climbot_control::firstScanEntryBudget(
    robot, first, second, {2.0, 0.03}, gravity, kSlipPerDegree, 0.05);
  EXPECT_NEAR(unreserved, 0.095, 1e-9);
  EXPECT_NEAR(reserved, 0.065, 1e-9) << "30 mm of lift should pay for 30 mm of drop";

  // Over-reserving cannot make the budget smaller than the tolerance itself:
  // the leftover is floored at zero rather than credited back.
  const double over = climbot_control::firstScanEntryBudget(
    robot, first, second, {2.0, 5.0}, gravity, kSlipPerDegree, 0.05);
  EXPECT_NEAR(over, 0.05, 1e-9);
}

TEST(ScanEntry, ATurnThatDropsWhatThePredictionSaysIsNotReported)
{
  const double degrees = 90.0;
  const double radians = degrees * 3.14159265358979323846 / 180.0;
  const double predicted = kSlipPerDegree * degrees;
  EXPECT_FALSE(climbot_control::turnSlipLooksStale(radians, predicted, kSlipPerDegree));
  // Within half the prediction is still agreement.
  EXPECT_FALSE(
    climbot_control::turnSlipLooksStale(radians, predicted * 1.4, kSlipPerDegree));
  EXPECT_TRUE(
    climbot_control::turnSlipLooksStale(radians, predicted * 4.0, kSlipPerDegree));
}

TEST(ScanEntry, SmallTurnsAreNeverJudged)
{
  // Five degrees predicts 2.5 mm of drop. Nothing measured over that distance
  // distinguishes a stale constant from noise, and warning about it would
  // teach an operator to ignore the warning.
  const double radians = 5.0 * 3.14159265358979323846 / 180.0;
  EXPECT_FALSE(climbot_control::turnSlipLooksStale(radians, 0.05, kSlipPerDegree));
}
