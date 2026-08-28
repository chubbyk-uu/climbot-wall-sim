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

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

#include "climbot_gazebo/wall_imu_adapter.hpp"
#include "gtest/gtest.h"

namespace
{

TEST(WallImuAdapter, PreservesMeasurementsAndPublishesTheEkfCovarianceContract)
{
  sensor_msgs::msg::Imu source;
  source.header.stamp.sec = 7;
  source.header.stamp.nanosec = 123U;
  source.header.frame_id = "imu_link";
  source.orientation.w = 1.0;
  source.angular_velocity.x = 0.3;
  source.linear_acceleration.z = 9.81;
  source.angular_velocity_covariance[0U] = 0.04;
  source.linear_acceleration_covariance[8U] = 0.09;
  const climbot_gazebo::WallImuAdapterConfig config{0.0, 17};
  std::mt19937 random_engine(static_cast<std::mt19937::result_type>(config.random_seed));

  const auto adapted = climbot_gazebo::adaptWallImu(source, config, random_engine);

  EXPECT_EQ(adapted.header.stamp.sec, source.header.stamp.sec);
  EXPECT_EQ(adapted.header.stamp.nanosec, source.header.stamp.nanosec);
  EXPECT_EQ(adapted.header.frame_id, source.header.frame_id);
  EXPECT_DOUBLE_EQ(adapted.orientation.x, source.orientation.x);
  EXPECT_DOUBLE_EQ(adapted.orientation.y, source.orientation.y);
  EXPECT_DOUBLE_EQ(adapted.orientation.z, source.orientation.z);
  EXPECT_DOUBLE_EQ(adapted.orientation.w, source.orientation.w);
  EXPECT_DOUBLE_EQ(adapted.angular_velocity.x, source.angular_velocity.x);
  EXPECT_DOUBLE_EQ(adapted.linear_acceleration.z, source.linear_acceleration.z);
  EXPECT_DOUBLE_EQ(adapted.angular_velocity_covariance[0U], 0.04);
  EXPECT_DOUBLE_EQ(adapted.linear_acceleration_covariance[8U], 0.09);
  for (const auto index : {0U, 4U, 8U}) {
    EXPECT_DOUBLE_EQ(adapted.orientation_covariance[index], 0.0);
  }
}

TEST(WallImuAdapter, AddsNormalizedDeterministicAttitudeNoise)
{
  sensor_msgs::msg::Imu source;
  source.orientation.w = 1.0;
  const climbot_gazebo::WallImuAdapterConfig config{0.01, 23};
  std::mt19937 first_engine(static_cast<std::mt19937::result_type>(config.random_seed));
  std::mt19937 second_engine(static_cast<std::mt19937::result_type>(config.random_seed));

  const auto first = climbot_gazebo::adaptWallImu(source, config, first_engine);
  const auto second = climbot_gazebo::adaptWallImu(source, config, second_engine);
  const double norm = std::sqrt(
    first.orientation.x * first.orientation.x + first.orientation.y * first.orientation.y +
    first.orientation.z * first.orientation.z + first.orientation.w * first.orientation.w);

  EXPECT_DOUBLE_EQ(first.orientation.x, second.orientation.x);
  EXPECT_DOUBLE_EQ(first.orientation.y, second.orientation.y);
  EXPECT_DOUBLE_EQ(first.orientation.z, second.orientation.z);
  EXPECT_DOUBLE_EQ(first.orientation.w, second.orientation.w);
  EXPECT_NE(first.orientation.x, 0.0);
  EXPECT_NEAR(norm, 1.0, 1e-12);
  for (const auto index : {0U, 4U, 8U}) {
    EXPECT_DOUBLE_EQ(first.orientation_covariance[index], 0.0001);
  }
}

TEST(WallImuAdapter, RejectsInvalidParameters)
{
  EXPECT_THROW(
    climbot_gazebo::validateWallImuAdapterConfig({-0.01, 17}), std::invalid_argument);
  EXPECT_THROW(
    climbot_gazebo::validateWallImuAdapterConfig(
      {std::numeric_limits<double>::quiet_NaN(), 17}), std::invalid_argument);
}

}  // namespace
