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

#include <memory>
#include <random>

#include "climbot_gazebo/wall_imu_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class WallImuAdapter : public rclcpp::Node
{
public:
  WallImuAdapter()
  : Node("wall_imu_adapter")
  {
    config_.orientation_stddev_rad = declare_parameter(
      "orientation_stddev_rad", 0.0017453292519943296);
    config_.random_seed = declare_parameter("random_seed", int64_t{17});
    climbot_gazebo::validateWallImuAdapterConfig(config_);
    random_engine_.seed(static_cast<std::mt19937::result_type>(config_.random_seed));
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu_wall", 50);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 50, [this](const sensor_msgs::msg::Imu::SharedPtr source) {
        publisher_->publish(climbot_gazebo::adaptWallImu(*source, config_, random_engine_));
      });
  }

private:
  climbot_gazebo::WallImuAdapterConfig config_;
  std::mt19937 random_engine_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WallImuAdapter>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("wall_imu_adapter"), "Node failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
