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

#include "climbot_gazebo/wall_wheel_odom_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class WallWheelOdomAdapter : public rclcpp::Node
{
public:
  WallWheelOdomAdapter()
  : Node("wall_wheel_odom_adapter")
  {
    config_.forward_velocity_stddev_mps =
      declare_parameter("forward_velocity_stddev_mps", 0.03);
    config_.yaw_rate_stddev_rps = declare_parameter("yaw_rate_stddev_rps", 0.05);
    config_.unobserved_variance = declare_parameter("unobserved_variance", 1e6);
    climbot_gazebo::validateWheelOdomAdapterConfig(config_);
    publisher_ = create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 20);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/model/climbot/odometry", 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr source) {
        publisher_->publish(climbot_gazebo::adaptWallWheelOdometry(*source, config_));
      });
  }

private:
  climbot_gazebo::WheelOdomAdapterConfig config_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WallWheelOdomAdapter>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("wall_wheel_odom_adapter"), "Node failed: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
