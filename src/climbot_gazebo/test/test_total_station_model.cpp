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

#include <cmath>
#include <stdexcept>

#include "climbot_gazebo/total_station_model.hpp"

namespace
{
constexpr double kHalfPi = 1.57079632679489661923;
}  // namespace

TEST(TotalStationModel, AutoFollowsTheProfileAndOverridesWin)
{
  EXPECT_FALSE(climbot_gazebo::resolveComponentEnabled("precision", "auto"));
  EXPECT_TRUE(climbot_gazebo::resolveComponentEnabled("realistic", "auto"));
  EXPECT_TRUE(climbot_gazebo::resolveComponentEnabled("precision", "enabled"));
  EXPECT_FALSE(climbot_gazebo::resolveComponentEnabled("realistic", "disabled"));
}

TEST(TotalStationModel, RejectsUnknownProfilesAndModes)
{
  EXPECT_THROW(
    climbot_gazebo::resolveComponentEnabled("approximate", "auto"), std::invalid_argument);
  EXPECT_THROW(
    climbot_gazebo::resolveComponentEnabled("precision", "sometimes"), std::invalid_argument);
}

TEST(TotalStationModel, ResidualReversesWhenTheRobotReverses)
{
  // The point of rotating into the wall plane: a forward-mounted prism offset
  // must push the measurement the other way once the robot turns around.
  const std::array<double, 3> residual{0.02, 0.0, 0.0};
  const auto forward = climbot_gazebo::rotateRobotResidualToWall(residual, 0.0);
  const auto reversed = climbot_gazebo::rotateRobotResidualToWall(residual, 2.0 * kHalfPi);
  EXPECT_NEAR(forward[0], 0.02, 1e-12);
  EXPECT_NEAR(reversed[0], -0.02, 1e-12);
}

TEST(TotalStationModel, ResidualRotatesLateralIntoTheWallPlane)
{
  const std::array<double, 3> residual{0.0, 0.01, 0.003};
  const auto rotated = climbot_gazebo::rotateRobotResidualToWall(residual, kHalfPi);
  EXPECT_NEAR(rotated[0], -0.01, 1e-12);
  EXPECT_NEAR(rotated[1], 0.0, 1e-12);
  // The wall normal is shared, so the third component never rotates.
  EXPECT_NEAR(rotated[2], 0.003, 1e-12);
}

TEST(TotalStationModel, AppliesAClockCorrectionInNanoseconds)
{
  EXPECT_EQ(climbot_gazebo::timestampWithClockErrorNs(1'000'000'000LL, 0.02), 1'020'000'000LL);
  EXPECT_EQ(climbot_gazebo::timestampWithClockErrorNs(1'000'000'000LL, -0.02), 980'000'000LL);
}

TEST(TotalStationModel, ClampsANegativeStampToZero)
{
  // builtin_interfaces/Time has no negative nanosecond representation.
  EXPECT_EQ(climbot_gazebo::timestampWithClockErrorNs(1'000'000LL, -5.0), 0LL);
}
