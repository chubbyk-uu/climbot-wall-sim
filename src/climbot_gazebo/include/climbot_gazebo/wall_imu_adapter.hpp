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

#ifndef CLIMBOT_GAZEBO__WALL_IMU_ADAPTER_HPP_
#define CLIMBOT_GAZEBO__WALL_IMU_ADAPTER_HPP_

#include <cstdint>
#include <random>

#include "sensor_msgs/msg/imu.hpp"

namespace climbot_gazebo
{

struct WallImuAdapterConfig
{
  double orientation_stddev_rad{0.0017453292519943296};
  int64_t random_seed{17};
};

/// Validate the stable parameter contract shared with the former Python node.
void validateWallImuAdapterConfig(const WallImuAdapterConfig & config);

/// Copy an IMU observation and inject independent RPY attitude uncertainty.
/// The supplied engine is intentionally owned by the node so every input
/// observation advances the deterministic noise sequence exactly once.
sensor_msgs::msg::Imu adaptWallImu(
  const sensor_msgs::msg::Imu & source, const WallImuAdapterConfig & config,
  std::mt19937 & random_engine);

}  // namespace climbot_gazebo

#endif  // CLIMBOT_GAZEBO__WALL_IMU_ADAPTER_HPP_
