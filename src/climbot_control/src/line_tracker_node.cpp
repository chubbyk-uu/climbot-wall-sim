#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "climbot_control/line_tracker.hpp"
#include "climbot_control/turn_profile.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

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
    cross_integral_gain_ = declare_parameter("cross_integral_gain", 0.30);
    cross_integral_limit_ = declare_parameter("cross_integral_limit_m_s", 0.10);
    heading_gain_ = declare_parameter("heading_gain", 2.0);
    control_frequency_hz_ = declare_parameter("control_frequency_hz", 50.0);
    odometry_timeout_s_ = declare_parameter("odometry_timeout_s", 0.25);
    frame_id_ = declare_parameter("frame_id", "odom");

    limits_.max_linear = declare_parameter("max_linear_speed", 0.15);
    limits_.max_angular = declare_parameter("max_angular_speed", 0.35);
    limits_.max_heading_correction = degreesToRadians(
      declare_parameter("max_heading_correction_deg", 12.0));
    limits_.max_gravity_feedforward = degreesToRadians(
      declare_parameter("max_gravity_feedforward_deg", 8.0));
    limits_.max_cross_feedback = degreesToRadians(
      declare_parameter("max_cross_feedback_deg", 8.0));
    limits_.alignment_threshold = degreesToRadians(
      declare_parameter("alignment_threshold_deg", 10.0));
    alignment_reentry_threshold_ = degreesToRadians(
      declare_parameter("alignment_reentry_threshold_deg", 12.0));
    alignment_tolerance_ = degreesToRadians(
      declare_parameter("alignment_tolerance_deg", 2.0));
    alignment_settle_duration_ = declare_parameter("alignment_settle_duration_s", 0.50);
    turn_heading_gain_ = declare_parameter("turn_heading_gain", 2.0);
    max_turn_angular_speed_ = declare_parameter("max_turn_angular_speed", 0.60);
    max_turn_angular_acceleration_ = declare_parameter(
      "max_turn_angular_acceleration", 1.00);
    final_approach_distance_ = declare_parameter("final_approach_distance_m", 0.10);
    final_approach_speed_ = declare_parameter("final_approach_speed_mps", 0.03);
    goal_position_tolerance_ = declare_parameter("goal_position_tolerance_m", 0.03);
    goal_position_exit_tolerance_ = declare_parameter(
      "goal_position_exit_tolerance_m", 0.04);
    goal_heading_exit_tolerance_ = degreesToRadians(
      declare_parameter("goal_heading_exit_tolerance_deg", 3.0));
    stopped_linear_speed_ = declare_parameter("stopped_linear_speed_mps", 0.01);
    stopped_angular_speed_ = declare_parameter("stopped_angular_speed_rps", 0.02);
    goal_settle_duration_ = declare_parameter("goal_settle_duration_s", 0.30);
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
    completion_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/control/segment_complete", rclcpp::QoS(1).reliable().transient_local());
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
        const auto & linear = message->twist.twist.linear;
        const auto & angular = message->twist.twist.angular;
        if (!std::isfinite(linear.x) || !std::isfinite(linear.y) ||
        !std::isfinite(angular.z))
        {
          RCLCPP_ERROR(get_logger(), "Rejected invalid filtered odometry velocity.");
          return;
        }
        pose_ = {
          position.x,
          position.y,
          *yaw};
        measured_linear_speed_ = std::hypot(linear.x, linear.y);
        measured_angular_speed_ = std::abs(angular.z);
        last_pose_received_time_ = now();
        have_pose_ = true;
      });

    publishReferencePath();
    publishCompletion(false);
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
    requireFinite("cross_integral_gain", cross_integral_gain_);
    if (cross_integral_gain_ < 0.0) {
      throw std::invalid_argument("cross_integral_gain must be non-negative.");
    }
    requirePositive("cross_integral_limit_m_s", cross_integral_limit_);
    requirePositive("heading_gain", heading_gain_);
    requirePositive("control_frequency_hz", control_frequency_hz_);
    requirePositive("odometry_timeout_s", odometry_timeout_s_);
    requirePositive("max_linear_speed", limits_.max_linear);
    requirePositive("max_angular_speed", limits_.max_angular);
    requirePositive("max_heading_correction_deg", limits_.max_heading_correction);
    requirePositive("max_gravity_feedforward_deg", limits_.max_gravity_feedforward);
    requirePositive("max_cross_feedback_deg", limits_.max_cross_feedback);
    requirePositive("alignment_threshold_deg", limits_.alignment_threshold);
    requirePositive("alignment_reentry_threshold_deg", alignment_reentry_threshold_);
    requirePositive("alignment_tolerance_deg", alignment_tolerance_);
    requirePositive("alignment_settle_duration_s", alignment_settle_duration_);
    requirePositive("turn_heading_gain", turn_heading_gain_);
    requirePositive("max_turn_angular_speed", max_turn_angular_speed_);
    requirePositive("max_turn_angular_acceleration", max_turn_angular_acceleration_);
    requirePositive("final_approach_distance_m", final_approach_distance_);
    requirePositive("final_approach_speed_mps", final_approach_speed_);
    requirePositive("goal_position_tolerance_m", goal_position_tolerance_);
    requirePositive("goal_position_exit_tolerance_m", goal_position_exit_tolerance_);
    requirePositive("goal_heading_exit_tolerance_deg", goal_heading_exit_tolerance_);
    requirePositive("stopped_linear_speed_mps", stopped_linear_speed_);
    requirePositive("stopped_angular_speed_rps", stopped_angular_speed_);
    requirePositive("goal_settle_duration_s", goal_settle_duration_);
    if (alignment_tolerance_ >= limits_.alignment_threshold) {
      throw std::invalid_argument(
              "alignment_tolerance_deg must be below alignment_threshold_deg.");
    }
    if (alignment_reentry_threshold_ <= limits_.alignment_threshold) {
      throw std::invalid_argument(
              "alignment_reentry_threshold_deg must exceed alignment_threshold_deg.");
    }
    if (final_approach_speed_ > limits_.max_linear) {
      throw std::invalid_argument(
              "final_approach_speed_mps cannot exceed max_linear_speed.");
    }
    if (goal_position_exit_tolerance_ <= goal_position_tolerance_) {
      throw std::invalid_argument(
              "goal_position_exit_tolerance_m must exceed goal_position_tolerance_m.");
    }
    if (final_approach_distance_ <= goal_position_exit_tolerance_) {
      throw std::invalid_argument(
              "final_approach_distance_m must exceed goal_position_exit_tolerance_m.");
    }
    if (goal_heading_exit_tolerance_ <= alignment_tolerance_) {
      throw std::invalid_argument(
              "goal_heading_exit_tolerance_deg must exceed alignment_tolerance_deg.");
    }
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

  void publishCompletion(bool complete)
  {
    std_msgs::msg::Bool message;
    message.data = complete;
    completion_publisher_->publish(message);
  }

  void onTimer()
  {
    const auto current_time = now();
    if (!have_pose_ || current_time < last_pose_received_time_ ||
      (current_time - last_pose_received_time_).seconds() > odometry_timeout_s_)
    {
      previous_command_ = {};
      cross_integral_ = 0.0;
      if (motion_state_ != MotionState::SEGMENT_COMPLETE) {
        motion_state_ = MotionState::WAITING_FOR_ALIGNMENT;
      }
      alignment_settle_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      goal_settle_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
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

    auto desired = climbot_control::trackLine(start_, end_, pose_, cruise_speed_,
        cross_gain_, heading_gain_, limits_, cross_integral_gain_, cross_integral_);
    if (motion_state_ == MotionState::SEGMENT_COMPLETE) {
      limitAndPublish({}, dt, angular_acceleration_);
      return;
    }
    if (motion_state_ != MotionState::TRACK_LINE &&
      motion_state_ != MotionState::FINAL_APPROACH)
    {
      updateAlignment(current_time, dt, desired);
      return;
    }
    if (std::abs(desired.heading_error) >= alignment_reentry_threshold_) {
      cross_integral_ = 0.0;
      motion_state_ = MotionState::ALIGN_BRAKE;
      updateAlignment(current_time, dt, desired);
      return;
    }
    if (motion_state_ == MotionState::TRACK_LINE &&
      desired.remaining <= final_approach_distance_)
    {
      motion_state_ = MotionState::FINAL_APPROACH;
    }
    if (motion_state_ == MotionState::FINAL_APPROACH) {
      desired.linear = std::min(desired.linear, final_approach_speed_);
      if (updateGoalCompletion(current_time, desired)) {
        limitAndPublish({}, dt, angular_acceleration_);
        return;
      }
    }
    if (desired.linear > 0.0 && cross_integral_gain_ > 0.0) {
      const double candidate_integral = std::clamp(
        cross_integral_ + desired.cross * dt, -cross_integral_limit_, cross_integral_limit_);
      const auto candidate = climbot_control::trackLine(start_, end_, pose_, cruise_speed_,
          cross_gain_, heading_gain_, limits_, cross_integral_gain_, candidate_integral);
      const double integral_drive = -cross_integral_gain_ *
        (candidate_integral - cross_integral_);
      constexpr double saturation_tolerance = 1e-12;
      const bool feedback_at_limit =
        std::abs(candidate.cross_feedback) >=
        limits_.max_cross_feedback - saturation_tolerance;
      const bool total_at_limit =
        std::abs(candidate.heading_correction) >=
        limits_.max_heading_correction - saturation_tolerance;
      const bool drives_further_into_saturation =
        (feedback_at_limit && candidate.cross_feedback * integral_drive > 0.0) ||
        (total_at_limit && candidate.heading_correction * integral_drive > 0.0);
      if (!drives_further_into_saturation) {
        cross_integral_ = candidate_integral;
        desired = candidate;
      }
    }
    limitAndPublish(desired, dt, angular_acceleration_);
  }

  bool updateGoalCompletion(
    const rclcpp::Time & current_time, const climbot_control::Command & command)
  {
    const double position_error = std::hypot(end_.x - pose_.x, end_.y - pose_.y);
    const double heading_error = std::abs(command.heading_error);
    const bool stopped = measured_linear_speed_ <= stopped_linear_speed_ &&
      measured_angular_speed_ <= stopped_angular_speed_;
    const bool strict_goal = position_error <= goal_position_tolerance_ &&
      heading_error <= alignment_tolerance_ && stopped;
    const bool relaxed_goal = position_error <= goal_position_exit_tolerance_ &&
      heading_error <= goal_heading_exit_tolerance_ && stopped;

    if (!relaxed_goal) {
      goal_settle_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return false;
    }
    if (!strict_goal) {
      return false;
    }
    if (goal_settle_start_.nanoseconds() == 0) {
      goal_settle_start_ = current_time;
      return false;
    }
    if ((current_time - goal_settle_start_).seconds() < goal_settle_duration_) {
      return false;
    }

    motion_state_ = MotionState::SEGMENT_COMPLETE;
    cross_integral_ = 0.0;
    publishCompletion(true);
    RCLCPP_INFO(
      get_logger(), "Segment complete: position error %.4f m, heading error %.2f deg.",
      position_error, heading_error * 180.0 / std::acos(-1.0));
    return true;
  }

  void updateAlignment(
    const rclcpp::Time & current_time, double dt,
    const climbot_control::Command & line_command)
  {
    climbot_control::Command command;
    switch (motion_state_) {
      case MotionState::WAITING_FOR_ALIGNMENT:
        motion_state_ = MotionState::ALIGN_BRAKE;
        break;
      case MotionState::ALIGN_BRAKE:
        break;
      case MotionState::ALIGN_PROFILE:
        {
          const double elapsed = (current_time - alignment_profile_start_).seconds();
          const auto sample = climbot_control::sampleTurn(alignment_profile_, elapsed);
          const double reference_yaw = alignment_start_yaw_ + sample.angle;
          const double error = climbot_control::wrapAngle(reference_yaw - pose_.yaw);
          command.angular = std::clamp(
          sample.angular_rate + turn_heading_gain_ * error,
          -max_turn_angular_speed_, max_turn_angular_speed_);
          limitAndPublish(command, dt, max_turn_angular_acceleration_);
          if (elapsed >= alignment_profile_.duration) {
            motion_state_ = MotionState::ALIGN_SETTLE;
            alignment_settle_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          }
          return;
        }
      case MotionState::ALIGN_SETTLE:
        {
          if (std::abs(line_command.heading_error) <= alignment_tolerance_) {
            command.angular = 0.0;
          } else {
            alignment_settle_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            command.angular = std::clamp(
            turn_heading_gain_ * line_command.heading_error,
            -max_turn_angular_speed_, max_turn_angular_speed_);
          }
          limitAndPublish(command, dt, max_turn_angular_acceleration_);
          if (std::abs(line_command.heading_error) <= alignment_tolerance_ &&
            std::abs(previous_command_.angular) <= 1e-3)
          {
            if (alignment_settle_start_.nanoseconds() == 0) {
              alignment_settle_start_ = current_time;
            } else if ((current_time - alignment_settle_start_).seconds() >=
              alignment_settle_duration_)
            {
              motion_state_ = MotionState::TRACK_LINE;
              cross_integral_ = 0.0;
            }
          }
          return;
        }
      case MotionState::TRACK_LINE:
      case MotionState::FINAL_APPROACH:
      case MotionState::SEGMENT_COMPLETE:
        return;
    }

    limitAndPublish(command, dt, max_turn_angular_acceleration_);
    if (std::abs(previous_command_.linear) <= 1e-4 &&
      std::abs(previous_command_.angular) <= 1e-3)
    {
      alignment_start_yaw_ = pose_.yaw;
      alignment_profile_ = climbot_control::planTurn(
        line_command.heading_error, max_turn_angular_speed_,
        max_turn_angular_acceleration_);
      alignment_profile_start_ = current_time;
      alignment_settle_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      motion_state_ = alignment_profile_.duration > 0.0 ?
        MotionState::ALIGN_PROFILE : MotionState::ALIGN_SETTLE;
    }
  }

  void limitAndPublish(
    const climbot_control::Command & desired, double dt, double angular_acceleration)
  {
    previous_command_ = climbot_control::rateLimit(
      desired, previous_command_, dt, linear_acceleration_, limits_.max_deceleration,
      angular_acceleration, wheel_separation_, wheel_speed_limit_,
      wheel_acceleration_limit_);
    geometry_msgs::msg::Twist command;
    command.linear.x = previous_command_.linear;
    command.angular.z = previous_command_.angular;
    command_publisher_->publish(command);
  }

  enum class MotionState
  {
    WAITING_FOR_ALIGNMENT,
    ALIGN_BRAKE,
    ALIGN_PROFILE,
    ALIGN_SETTLE,
    TRACK_LINE,
    FINAL_APPROACH,
    SEGMENT_COMPLETE,
  };

  bool have_pose_{false};
  double cruise_speed_{0.15};
  double cross_gain_{1.0};
  double cross_integral_gain_{0.30};
  double cross_integral_limit_{0.10};
  double cross_integral_{0.0};
  double heading_gain_{2.0};
  double control_frequency_hz_{50.0};
  double odometry_timeout_s_{0.25};
  double alignment_reentry_threshold_{0.209439510};
  double alignment_tolerance_{0.034906585};
  double alignment_settle_duration_{0.50};
  double turn_heading_gain_{2.0};
  double max_turn_angular_speed_{0.60};
  double max_turn_angular_acceleration_{1.00};
  double final_approach_distance_{0.10};
  double final_approach_speed_{0.03};
  double goal_position_tolerance_{0.03};
  double goal_position_exit_tolerance_{0.04};
  double goal_heading_exit_tolerance_{0.052359878};
  double stopped_linear_speed_{0.01};
  double stopped_angular_speed_{0.02};
  double goal_settle_duration_{0.30};
  double measured_linear_speed_{0.0};
  double measured_angular_speed_{0.0};
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
  MotionState motion_state_{MotionState::WAITING_FOR_ALIGNMENT};
  climbot_control::TurnProfile alignment_profile_;
  double alignment_start_yaw_{0.0};
  rclcpp::Time alignment_profile_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time alignment_settle_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_settle_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_publisher_;
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
