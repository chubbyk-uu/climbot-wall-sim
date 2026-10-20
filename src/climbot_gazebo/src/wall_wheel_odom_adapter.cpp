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

#include "climbot_gazebo/wall_wheel_odom_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_gazebo
{

void validateWheelOdomAdapterConfig(const WheelOdomAdapterConfig & config)
{
  const auto finite = [](double value) {return std::isfinite(value);};
  if (!finite(config.forward_velocity_stddev_mps) ||
    !finite(config.yaw_rate_stddev_rps) || !finite(config.unobserved_variance) ||
    !finite(config.gravity_creep_uphill_ratio) || !finite(config.gravity_creep_downhill_ratio))
  {
    throw std::invalid_argument("Wheel odometry adapter parameters must be finite.");
  }
  if (config.forward_velocity_stddev_mps < 0.0 || config.yaw_rate_stddev_rps < 0.0) {
    throw std::invalid_argument("Wheel odometry standard deviations cannot be negative.");
  }
  if (config.unobserved_variance <= 0.0) {
    throw std::invalid_argument("unobserved_variance must be positive.");
  }
  // A ratio this far from one is wheel spin, not creep.
  for (const auto ratio :
    {config.gravity_creep_uphill_ratio, config.gravity_creep_downhill_ratio})
  {
    if (ratio < 0.5 || ratio > 2.0) {
      throw std::invalid_argument("Gravity creep ratios must lie in [0.5, 2].");
    }
  }
}

double forwardUpComponent(const geometry_msgs::msg::Quaternion & q)
{
  // Rotation matrix entry R(1, 0): wall-frame Y of the body x axis.
  return std::clamp(2.0 * (q.x * q.y + q.w * q.z), -1.0, 1.0);
}

double gravityCreepRatio(double uphill_component, const WheelOdomAdapterConfig & config)
{
  const double up = std::clamp(uphill_component, -1.0, 1.0);
  return 1.0 + (config.gravity_creep_uphill_ratio - 1.0) * std::max(up, 0.0) +
         (config.gravity_creep_downhill_ratio - 1.0) * std::max(-up, 0.0);
}

nav_msgs::msg::Odometry adaptWallWheelOdometry(
  const nav_msgs::msg::Odometry & source, const WheelOdomAdapterConfig & config,
  double forward_up_component)
{
  validateWheelOdomAdapterConfig(config);
  nav_msgs::msg::Odometry adapted;
  adapted.header = source.header;
  adapted.child_frame_id = source.child_frame_id;
  adapted.pose.pose = source.pose.pose;
  adapted.twist.twist = source.twist.twist;
  // Creep follows the direction of travel: reversing up the wall climbs too.
  const double forward = source.twist.twist.linear.x;
  const double travel_up = forward >= 0.0 ? forward_up_component : -forward_up_component;
  adapted.twist.twist.linear.x = forward / gravityCreepRatio(travel_up, config);

  for (const auto index : {0U, 7U, 14U, 21U, 28U, 35U}) {
    adapted.pose.covariance[index] = config.unobserved_variance;
  }
  for (const auto index : {7U, 14U, 21U, 28U}) {
    adapted.twist.covariance[index] = config.unobserved_variance;
  }
  adapted.twist.covariance[0U] =
    config.forward_velocity_stddev_mps * config.forward_velocity_stddev_mps;
  adapted.twist.covariance[35U] = config.yaw_rate_stddev_rps * config.yaw_rate_stddev_rps;
  return adapted;
}

}  // namespace climbot_gazebo
