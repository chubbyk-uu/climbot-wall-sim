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

#ifndef CLIMBOT_GAZEBO__WALL_WHEEL_ODOM_ADAPTER_HPP_
#define CLIMBOT_GAZEBO__WALL_WHEEL_ODOM_ADAPTER_HPP_

#include "nav_msgs/msg/odometry.hpp"

namespace climbot_gazebo
{

struct WheelOdomAdapterConfig
{
  double forward_velocity_stddev_mps{0.03};
  double yaw_rate_stddev_rps{0.05};
  double unobserved_variance{1e6};
};

/// Validate the stable parameter contract shared with the former Python node.
void validateWheelOdomAdapterConfig(const WheelOdomAdapterConfig & config);

/// Preserve the raw observation while expressing the wall-slip uncertainty
/// consumed by robot_localization (twist.x and twist.angular.z only).
nav_msgs::msg::Odometry adaptWallWheelOdometry(
  const nav_msgs::msg::Odometry & source, const WheelOdomAdapterConfig & config);

}  // namespace climbot_gazebo

#endif  // CLIMBOT_GAZEBO__WALL_WHEEL_ODOM_ADAPTER_HPP_
