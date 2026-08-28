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

#include "climbot_gazebo/wall_imu_adapter.hpp"

#include <cmath>
#include <stdexcept>

namespace climbot_gazebo
{
namespace
{

geometry_msgs::msg::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  const double half_roll = roll * 0.5;
  const double half_pitch = pitch * 0.5;
  const double half_yaw = yaw * 0.5;
  const double cos_roll = std::cos(half_roll);
  const double sin_roll = std::sin(half_roll);
  const double cos_pitch = std::cos(half_pitch);
  const double sin_pitch = std::sin(half_pitch);
  const double cos_yaw = std::cos(half_yaw);
  const double sin_yaw = std::sin(half_yaw);

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
  quaternion.y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
  quaternion.z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
  quaternion.w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
  return quaternion;
}

geometry_msgs::msg::Quaternion multiplyQuaternions(
  const geometry_msgs::msg::Quaternion & left, const geometry_msgs::msg::Quaternion & right)
{
  geometry_msgs::msg::Quaternion product;
  product.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y;
  product.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x;
  product.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w;
  product.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
  return product;
}

}  // namespace

void validateWallImuAdapterConfig(const WallImuAdapterConfig & config)
{
  if (!std::isfinite(config.orientation_stddev_rad)) {
    throw std::invalid_argument("orientation_stddev_rad must be finite.");
  }
  if (config.orientation_stddev_rad < 0.0) {
    throw std::invalid_argument("orientation_stddev_rad cannot be negative.");
  }
}

sensor_msgs::msg::Imu adaptWallImu(
  const sensor_msgs::msg::Imu & source, const WallImuAdapterConfig & config,
  std::mt19937 & random_engine)
{
  validateWallImuAdapterConfig(config);
  sensor_msgs::msg::Imu adapted;
  adapted.header = source.header;
  adapted.angular_velocity = source.angular_velocity;
  adapted.linear_acceleration = source.linear_acceleration;
  adapted.angular_velocity_covariance = source.angular_velocity_covariance;
  adapted.linear_acceleration_covariance = source.linear_acceleration_covariance;

  std::normal_distribution<double> noise(0.0, config.orientation_stddev_rad);
  const auto attitude_noise = quaternionFromRpy(
    noise(random_engine), noise(random_engine), noise(random_engine));
  adapted.orientation = multiplyQuaternions(source.orientation, attitude_noise);

  const double variance = config.orientation_stddev_rad * config.orientation_stddev_rad;
  adapted.orientation_covariance[0U] = variance;
  adapted.orientation_covariance[4U] = variance;
  adapted.orientation_covariance[8U] = variance;
  return adapted;
}

}  // namespace climbot_gazebo
