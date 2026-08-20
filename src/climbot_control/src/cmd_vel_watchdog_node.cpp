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

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "climbot_control/command_watchdog.hpp"
#include "climbot_control/control_clock.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

class CmdVelWatchdogNode : public rclcpp::Node
{
public:
  CmdVelWatchdogNode()
  : Node("cmd_vel_watchdog"),
    watchdog_(declare_parameter("command_timeout_s", 0.40))
  {
    const double publish_rate_hz = declare_parameter("publish_rate_hz", 50.0);
    if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive and finite.");
    }
    // This node is the last thing between a stalled controller and the wheels,
    // so it must not share the controller's failure modes. On the node clock a
    // backward system-time step froze this timer for the length of the step
    // and, once it resumed, made the measured age of the last command smaller
    // rather than larger, so the one component whose job is to notice a missing
    // command was the last to notice it.
    control_clock_ = climbot_control::controlClock(this);
    output_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // The hold is the only way to stop the robot that does not go through the
    // executor or its Action server. Every other stop in this system is a
    // request to whatever is driving: cancel the goal, and the controller winds
    // the motion down. That is the right way when the controller is answering.
    // When it is not - the Action server has gone undiscoverable but
    // /control/cmd_vel is still being refreshed - there is nothing left to ask,
    // and the robot keeps driving a task nobody can call off. This sits below
    // all of that, on the last hop before the wheels, so it works whatever the
    // rest of the graph is doing.
    hold_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/control/hold_active", rclcpp::QoS(1).reliable().transient_local());
    hold_service_ = create_service<std_srvs::srv::SetBool>(
      "/control/hold",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
      const std_srvs::srv::SetBool::Response::SharedPtr response) {
        setHold(request->data);
        response->success = true;
        response->message = request->data ?
        "Holding: /cmd_vel is zero until the hold is released." :
        "Hold released: /control/cmd_vel drives /cmd_vel again.";
      });
    publishHold();
    input_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        if (!watchdog_.accept(
          {message->linear.x, message->angular.z}, control_clock_->now().seconds()))
        {
          RCLCPP_ERROR(get_logger(), "Rejected non-finite velocity command; stopping.");
        }
      });
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = rclcpp::create_timer(this, control_clock_, period,
      std::bind(&CmdVelWatchdogNode::publishCommand, this));
  }

private:
  void publishCommand()
  {
    geometry_msgs::msg::Twist output;
    if (!held_) {
      const auto filtered = watchdog_.commandAt(control_clock_->now().seconds());
      output.linear.x = filtered.linear;
      output.angular.z = filtered.angular;
    }
    // Held publishes the zero this default-constructs, at the same rate as any
    // other command. Silence would be a different thing: whatever consumes
    // /cmd_vel would be left to time the absence out on its own schedule, and
    // a hold has to be immediate.
    output_publisher_->publish(output);
  }

  void setHold(bool held)
  {
    if (held == held_) {
      return;
    }
    held_ = held;
    RCLCPP_WARN(
      get_logger(), held_ ?
      "Hold engaged: /cmd_vel forced to zero regardless of /control/cmd_vel." :
      "Hold released: /control/cmd_vel drives /cmd_vel again.");
    publishHold();
  }

  /// Latched, so anything that starts later still learns the robot is held.
  //
  // A hold nobody can see is a robot that will not move for no visible reason.
  // Whoever engaged it can crash or be restarted while it is engaged, and then
  // the state exists only here.
  void publishHold()
  {
    std_msgs::msg::Bool message;
    message.data = held_;
    hold_publisher_->publish(message);
  }

  climbot_control::CommandWatchdog watchdog_;
  rclcpp::Clock::SharedPtr control_clock_;
  bool held_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr hold_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CmdVelWatchdogNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("cmd_vel_watchdog"), "Startup failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
