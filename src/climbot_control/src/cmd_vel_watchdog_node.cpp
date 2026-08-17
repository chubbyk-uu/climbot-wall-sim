#include <chrono>
#include <memory>
#include <stdexcept>

#include "climbot_control/command_watchdog.hpp"
#include "climbot_control/control_clock.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelWatchdogNode : public rclcpp::Node
{
public:
  CmdVelWatchdogNode()
  : Node("cmd_vel_watchdog"),
    watchdog_(declare_parameter("command_timeout_s", 0.40))
  {
    const double publish_rate_hz = declare_parameter("publish_rate_hz", 50.0);
    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive.");
    }
    // This node is the last thing between a stalled controller and the wheels,
    // so it must not share the controller's failure modes. On the node clock a
    // backward system-time step froze this timer for the length of the step
    // and, once it resumed, made the measured age of the last command smaller
    // rather than larger, so the one component whose job is to notice a missing
    // command was the last to notice it.
    control_clock_ = climbot_control::controlClock(this);
    output_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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
    const auto filtered = watchdog_.commandAt(control_clock_->now().seconds());
    geometry_msgs::msg::Twist output;
    output.linear.x = filtered.linear;
    output.angular.z = filtered.angular;
    output_publisher_->publish(output);
  }

  climbot_control::CommandWatchdog watchdog_;
  rclcpp::Clock::SharedPtr control_clock_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_subscription_;
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
