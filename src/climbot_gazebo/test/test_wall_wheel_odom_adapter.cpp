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

#include <limits>

#include "climbot_gazebo/wall_wheel_odom_adapter.hpp"
#include "gtest/gtest.h"

namespace
{

TEST(WallWheelOdomAdapter, PreservesMeasurementsAndPublishesTheEkfCovarianceContract)
{
  nav_msgs::msg::Odometry source;
  source.header.stamp.sec = 7;
  source.header.stamp.nanosec = 123U;
  source.header.frame_id = "odom";
  source.child_frame_id = "base_link";
  source.pose.pose.position.x = 1.25;
  source.twist.twist.linear.x = 0.12;
  source.twist.twist.angular.z = -0.08;
  const climbot_gazebo::WheelOdomAdapterConfig config{0.03, 0.05, 1e6};

  const auto adapted = climbot_gazebo::adaptWallWheelOdometry(source, config);

  EXPECT_EQ(adapted.header.stamp.sec, source.header.stamp.sec);
  EXPECT_EQ(adapted.header.stamp.nanosec, source.header.stamp.nanosec);
  EXPECT_EQ(adapted.header.frame_id, source.header.frame_id);
  EXPECT_EQ(adapted.child_frame_id, source.child_frame_id);
  EXPECT_DOUBLE_EQ(adapted.pose.pose.position.x, source.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(adapted.twist.twist.linear.x, source.twist.twist.linear.x);
  EXPECT_DOUBLE_EQ(adapted.twist.twist.angular.z, source.twist.twist.angular.z);
  EXPECT_DOUBLE_EQ(adapted.twist.covariance[0U], 0.0009);
  EXPECT_DOUBLE_EQ(adapted.twist.covariance[35U], 0.0025);
  for (const auto index : {0U, 7U, 14U, 21U, 28U, 35U}) {
    EXPECT_DOUBLE_EQ(adapted.pose.covariance[index], 1e6);
  }
  for (const auto index : {7U, 14U, 21U, 28U}) {
    EXPECT_DOUBLE_EQ(adapted.twist.covariance[index], 1e6);
  }
}

TEST(WallWheelOdomAdapter, RejectsInvalidParameters)
{
  EXPECT_THROW(
    climbot_gazebo::validateWheelOdomAdapterConfig({-0.01, 0.05, 1e6}),
    std::invalid_argument);
  EXPECT_THROW(
    climbot_gazebo::validateWheelOdomAdapterConfig({0.03, 0.05, 0.0}),
    std::invalid_argument);
  EXPECT_THROW(
    climbot_gazebo::validateWheelOdomAdapterConfig(
      {std::numeric_limits<double>::quiet_NaN(), 0.05, 1e6}), std::invalid_argument);
}

}  // namespace
