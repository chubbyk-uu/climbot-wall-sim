#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "climbot_control/line_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class LineTrackerNode : public rclcpp::Node
{
public:
  LineTrackerNode()
  : Node("line_tracker")
  {
    start_ = {
      declare_parameter("start_x", 0.0), declare_parameter("start_y", 0.0)};
    end_ = {
      declare_parameter("end_x", 1.0), declare_parameter("end_y", 0.0)};
    cruise_speed_ = declare_parameter("cruise_speed", 0.15);
    cross_gain_ = declare_parameter("cross_gain", 1.0);
    heading_gain_ = declare_parameter("heading_gain", 2.0);
    control_frequency_hz_ = declare_parameter("control_frequency_hz", 50.0);
    frame_id_ = declare_parameter("frame_id", "odom");

    limits_.max_linear = declare_parameter("max_linear_speed", 0.15);
    limits_.max_angular = declare_parameter("max_angular_speed", 0.35);
    limits_.max_heading_correction = degreesToRadians(
      declare_parameter("max_heading_correction_deg", 10.0));
    limits_.alignment_threshold = degreesToRadians(
      declare_parameter("alignment_threshold_deg", 10.0));
    limits_.max_deceleration = declare_parameter("max_linear_deceleration", 0.25);
    limits_.gravity_slip_ratio = declare_parameter("gravity_slip_ratio", 0.0);
    limits_.gravity_direction = {
      declare_parameter("gravity_down_x", 0.0),
      declare_parameter("gravity_down_y", -1.0)};

    linear_acceleration_ = declare_parameter("max_linear_acceleration", 0.20);
    angular_acceleration_ = declare_parameter("max_angular_acceleration", 0.80);
    wheel_separation_ = declare_parameter("wheel_separation", 0.43);
    wheel_speed_limit_ = declare_parameter("wheel_speed_limit", 0.30);
    wheel_acceleration_limit_ = declare_parameter("wheel_acceleration_limit", 0.40);
    if (control_frequency_hz_ <= 0.0) {
      throw std::invalid_argument("control_frequency_hz must be positive.");
    }

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", 10);
    reference_publisher_ = create_publisher<nav_msgs::msg::Path>("/control/reference_path",
      rclcpp::QoS(1).reliable().transient_local());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        pose_ = {
          message->pose.pose.position.x,
          message->pose.pose.position.y,
          2.0 * std::atan2(message->pose.pose.orientation.z,
            message->pose.pose.orientation.w)};
        have_pose_ = true;
      });

    publishReferencePath();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_frequency_hz_));
    timer_ = rclcpp::create_timer(this, get_clock(), period,
      std::bind(&LineTrackerNode::onTimer, this));
  }

private:
  static double degreesToRadians(double degrees)
  {
    return degrees * std::acos(-1.0) / 180.0;
  }

  void publishReferencePath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = now();
    const double heading = std::atan2(end_.y - start_.y, end_.x - start_.x);
    for (const auto & point : {start_, end_}) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.z = std::sin(heading / 2.0);
      pose.pose.orientation.w = std::cos(heading / 2.0);
      path.poses.push_back(pose);
    }
    reference_publisher_->publish(path);
  }

  void onTimer()
  {
    if (!have_pose_) {
      return;
    }
    const auto current_time = now();
    const double dt = last_control_time_.nanoseconds() == 0 ?
      1.0 / control_frequency_hz_ : (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;
    if (dt <= 0.0) {
      return;
    }

    const auto desired = climbot_control::trackLine(start_, end_, pose_, cruise_speed_,
        cross_gain_, heading_gain_, limits_);
    previous_command_ = climbot_control::rateLimit(desired, previous_command_, dt,
        linear_acceleration_, limits_.max_deceleration, angular_acceleration_,
        wheel_separation_, wheel_speed_limit_, wheel_acceleration_limit_);

    geometry_msgs::msg::Twist command;
    command.linear.x = previous_command_.linear;
    command.angular.z = previous_command_.angular;
    command_publisher_->publish(command);
  }

  bool have_pose_{false};
  double cruise_speed_{0.15};
  double cross_gain_{1.0};
  double heading_gain_{2.0};
  double control_frequency_hz_{50.0};
  double linear_acceleration_{0.20};
  double angular_acceleration_{0.80};
  double wheel_separation_{0.43};
  double wheel_speed_limit_{0.30};
  double wheel_acceleration_limit_{0.40};
  std::string frame_id_{"odom"};
  climbot_control::Limits limits_;
  climbot_control::Point2 start_{};
  climbot_control::Point2 end_{};
  climbot_control::Pose2 pose_{};
  climbot_control::Command previous_command_;
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
