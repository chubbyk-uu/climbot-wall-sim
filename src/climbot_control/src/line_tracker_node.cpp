#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

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
    odometry_timeout_s_ = declare_parameter("odometry_timeout_s", 0.25);
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
    wheel_separation_ = declare_parameter("wheel_separation", -1.0);
    wheel_speed_limit_ = declare_parameter("wheel_speed_limit", -1.0);
    wheel_acceleration_limit_ = declare_parameter("wheel_acceleration_limit", -1.0);
    validateParameters();

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", 10);
    reference_publisher_ = create_publisher<nav_msgs::msg::Path>("/control/reference_path",
      rclcpp::QoS(1).reliable().transient_local());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        const auto & position = message->pose.pose.position;
        const auto & orientation = message->pose.pose.orientation;
        const auto yaw = climbot_control::yawFromQuaternion(
          orientation.x, orientation.y, orientation.z, orientation.w);
        if (!std::isfinite(position.x) || !std::isfinite(position.y) || !yaw) {
          RCLCPP_ERROR(get_logger(), "Rejected invalid filtered odometry pose.");
          return;
        }
        pose_ = {
          position.x,
          position.y,
          *yaw};
        last_pose_received_time_ = now();
        have_pose_ = true;
      });

    publishReferencePath();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_frequency_hz_));
    timer_ = rclcpp::create_timer(this, get_clock(), period,
      std::bind(&LineTrackerNode::onTimer, this));
  }

private:
  static void requireFinite(const std::string & name, double value)
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(name + " must be finite.");
    }
  }

  static void requirePositive(const std::string & name, double value)
  {
    requireFinite(name, value);
    if (value <= 0.0) {
      throw std::invalid_argument(name + " must be positive.");
    }
  }

  static double degreesToRadians(double degrees)
  {
    return degrees * std::acos(-1.0) / 180.0;
  }

  void validateParameters()
  {
    requireFinite("start_x", start_.x);
    requireFinite("start_y", start_.y);
    requireFinite("end_x", end_.x);
    requireFinite("end_y", end_.y);
    if (std::hypot(end_.x - start_.x, end_.y - start_.y) <= 1e-9) {
      throw std::invalid_argument("The line segment must have non-zero length.");
    }
    requirePositive("cruise_speed", cruise_speed_);
    requireFinite("cross_gain", cross_gain_);
    if (cross_gain_ < 0.0) {
      throw std::invalid_argument("cross_gain must be non-negative.");
    }
    requirePositive("heading_gain", heading_gain_);
    requirePositive("control_frequency_hz", control_frequency_hz_);
    requirePositive("odometry_timeout_s", odometry_timeout_s_);
    requirePositive("max_linear_speed", limits_.max_linear);
    requirePositive("max_angular_speed", limits_.max_angular);
    requirePositive("max_heading_correction_deg", limits_.max_heading_correction);
    requirePositive("alignment_threshold_deg", limits_.alignment_threshold);
    requirePositive("max_linear_deceleration", limits_.max_deceleration);
    requireFinite("gravity_slip_ratio", limits_.gravity_slip_ratio);
    if (limits_.gravity_slip_ratio < 0.0) {
      throw std::invalid_argument("gravity_slip_ratio must be non-negative.");
    }
    requireFinite("gravity_down_x", limits_.gravity_direction.x);
    requireFinite("gravity_down_y", limits_.gravity_direction.y);
    const double gravity_norm = std::hypot(
      limits_.gravity_direction.x, limits_.gravity_direction.y);
    if (gravity_norm <= 1e-9) {
      throw std::invalid_argument("The gravity direction must be non-zero.");
    }
    limits_.gravity_direction.x /= gravity_norm;
    limits_.gravity_direction.y /= gravity_norm;
    requirePositive("max_linear_acceleration", linear_acceleration_);
    requirePositive("max_angular_acceleration", angular_acceleration_);
    requirePositive("wheel_separation", wheel_separation_);
    requirePositive("wheel_speed_limit", wheel_speed_limit_);
    requirePositive("wheel_acceleration_limit", wheel_acceleration_limit_);
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id cannot be empty.");
    }
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
    const auto current_time = now();
    if (!have_pose_ || current_time < last_pose_received_time_ ||
      (current_time - last_pose_received_time_).seconds() > odometry_timeout_s_)
    {
      previous_command_ = {};
      last_control_time_ = current_time;
      geometry_msgs::msg::Twist stop;
      command_publisher_->publish(stop);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Filtered odometry is unavailable or stale; stopping.");
      return;
    }
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
  double odometry_timeout_s_{0.25};
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
  rclcpp::Time last_pose_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LineTrackerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("line_tracker"), "Failed to start: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
