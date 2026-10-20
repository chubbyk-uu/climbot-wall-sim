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
#include "sensor_msgs/msg/imu.hpp"

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
    config_.gravity_creep_uphill_ratio = declare_parameter("gravity_creep_uphill_ratio", 1.0);
    config_.gravity_creep_downhill_ratio =
      declare_parameter("gravity_creep_downhill_ratio", 1.0);
    climbot_gazebo::validateWheelOdomAdapterConfig(config_);
    publisher_ = create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 20);
    // The wall-frame attitude gives the gravity direction along the wheels.
    // Until the first sample arrives the robot is parked and creep is zero.
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu_wall", 50, [this](const sensor_msgs::msg::Imu::SharedPtr imu) {
        forward_up_component_ = climbot_gazebo::forwardUpComponent(imu->orientation);
      });
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/model/climbot/odometry", 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr source) {
        publisher_->publish(
          climbot_gazebo::adaptWallWheelOdometry(*source, config_, forward_up_component_));
      });
  }

private:
  climbot_gazebo::WheelOdomAdapterConfig config_;
  double forward_up_component_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
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
