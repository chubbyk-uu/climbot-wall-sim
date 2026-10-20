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

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace climbot_gazebo
{

struct WheelOdomAdapterConfig
{
  double forward_velocity_stddev_mps{0.03};
  double yaw_rate_stddev_rps{0.05};
  double unobserved_variance{1e6};
  // Wheel-to-true distance ratio when driving straight up (and down) the wall.
  // Longitudinal tyre slip ratio is set by traction over normal load, not by
  // speed, so climbing over-reads and descending under-reads by a fixed
  // fraction. One means no correction.
  double gravity_creep_uphill_ratio{1.0};
  double gravity_creep_downhill_ratio{1.0};
};

/// Validate the stable parameter contract shared with the former Python node.
void validateWheelOdomAdapterConfig(const WheelOdomAdapterConfig & config);

/// Preserve the raw observation while expressing the wall-slip uncertainty
/// consumed by robot_localization (twist.x and twist.angular.z only).
// Component of the body +x axis along wall +Y (up), from a wall-frame attitude.
double forwardUpComponent(const geometry_msgs::msg::Quaternion & wall_orientation);

// Odometry/true distance ratio for travel whose up component is uphill_component
// (+1 straight up, -1 straight down, 0 across the wall).
double gravityCreepRatio(double uphill_component, const WheelOdomAdapterConfig & config);

// forward_up_component is forwardUpComponent() of the current attitude; zero
// leaves the forward velocity unchanged.
nav_msgs::msg::Odometry adaptWallWheelOdometry(
  const nav_msgs::msg::Odometry & source, const WheelOdomAdapterConfig & config,
  double forward_up_component = 0.0);

}  // namespace climbot_gazebo

#endif  // CLIMBOT_GAZEBO__WALL_WHEEL_ODOM_ADAPTER_HPP_
