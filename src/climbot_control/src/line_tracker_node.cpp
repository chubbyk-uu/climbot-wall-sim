#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "climbot_control/control_clock.hpp"
#include "climbot_control/coverage_execution.hpp"
#include "climbot_control/line_tracker.hpp"
#include "climbot_control/segment_duration.hpp"
#include "climbot_control/travel_profile.hpp"
#include "climbot_control/turn_profile.hpp"
#include "climbot_interfaces/action/execute_coverage.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class LineTrackerNode : public rclcpp::Node
{
public:
  using ExecuteCoverage = climbot_interfaces::action::ExecuteCoverage;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCoverage>;

  enum class DynamicReferenceResult
  {
    FAILED,
    READY,
    REALIGN,
  };

  LineTrackerNode()
  : Node("line_tracker")
  {
    start_ = {
      declare_parameter("start_x", 0.0), declare_parameter("start_y", 0.0)};
    end_ = {
      declare_parameter("end_x", 1.0), declare_parameter("end_y", 0.0)};
    cruise_speed_ = declare_parameter("cruise_speed", 0.20);
    cross_gain_ = declare_parameter("cross_gain", 1.0);
    cross_integral_gain_ = declare_parameter("cross_integral_gain", 0.30);
    cross_integral_limit_ = declare_parameter("cross_integral_limit_m_s", 0.10);
    heading_gain_ = declare_parameter("heading_gain", 2.0);
    control_frequency_hz_ = declare_parameter("control_frequency_hz", 50.0);
    odometry_timeout_s_ = declare_parameter("odometry_timeout_s", 0.25);
    frame_id_ = declare_parameter("frame_id", "odom");
    standalone_mode_ = declare_parameter("standalone_mode", true);
    segment_timeout_s_ = declare_parameter("segment_timeout_s", 120.0);
    motion_region_tolerance_ = declare_parameter("motion_region_tolerance_m", 0.02);
    turn_slip_per_degree_ = declare_parameter("turn_slip_per_degree_m", 0.0005);
    parallel_scan_offset_ = declare_parameter("parallel_scan_offset_m", 0.045);
    maximum_scan_offset_ = declare_parameter("maximum_scan_offset_m", 0.12);
    arc_entry_finish_offset_ = declare_parameter("arc_entry_finish_offset_m", 0.012);
    arc_entry_speed_ = declare_parameter("arc_entry_speed_mps", 0.08);
    arc_entry_lookahead_ = declare_parameter("arc_entry_lookahead_m", 0.20);
    arc_entry_heading_gain_ = declare_parameter("arc_entry_heading_gain", 2.0);
    arc_entry_max_heading_ = degreesToRadians(
      declare_parameter("arc_entry_max_heading_deg", 20.0));
    arc_entry_max_angular_ = declare_parameter("arc_entry_max_angular_speed", 0.25);
    arc_entry_timeout_ = declare_parameter("arc_entry_timeout_s", 15.0);
    const int oscillation_reversals = declare_parameter("visible_oscillation_reversals", 4);
    if (oscillation_reversals < 0) {
      throw std::invalid_argument("visible_oscillation_reversals must be non-negative.");
    }
    oscillation_monitor_ = std::make_unique<climbot_control::CrossTrackOscillationMonitor>(
      declare_parameter("visible_oscillation_amplitude_m", 0.03),
      declare_parameter("visible_oscillation_minimum_travel_m", 0.10),
      static_cast<unsigned int>(oscillation_reversals));

    limits_.max_linear = declare_parameter("max_linear_speed", 0.25);
    limits_.max_angular = declare_parameter("max_angular_speed", 0.35);
    limits_.max_heading_correction = degreesToRadians(
      declare_parameter("max_heading_correction_deg", 12.0));
    limits_.max_gravity_feedforward = degreesToRadians(
      declare_parameter("max_gravity_feedforward_deg", 8.0));
    limits_.max_cross_feedback = degreesToRadians(
      declare_parameter("max_cross_feedback_deg", 8.0));
    limits_.alignment_threshold = degreesToRadians(
      declare_parameter("alignment_threshold_deg", 10.0));
    limits_.cross_slowdown_start = declare_parameter("cross_slowdown_start_m", 0.03);
    limits_.cross_slowdown_full = declare_parameter("cross_slowdown_full_m", 0.08);
    limits_.cross_slowdown_min_scale = declare_parameter("cross_slowdown_min_scale", 0.25);
    alignment_reentry_threshold_ = degreesToRadians(
      declare_parameter("alignment_reentry_threshold_deg", 12.0));
    alignment_tolerance_ = degreesToRadians(
      declare_parameter("alignment_tolerance_deg", 2.0));
    alignment_settle_duration_ = declare_parameter("alignment_settle_duration_s", 0.50);
    turn_heading_gain_ = declare_parameter("turn_heading_gain", 2.0);
    max_turn_angular_speed_ = declare_parameter("max_turn_angular_speed", 0.60);
    max_turn_angular_acceleration_ = declare_parameter(
      "max_turn_angular_acceleration", 1.00);
    final_approach_distance_ = declare_parameter("final_approach_distance_m", 0.05);
    final_approach_speed_ = declare_parameter("final_approach_speed_mps", 0.08);
    goal_position_tolerance_ = declare_parameter("goal_position_tolerance_m", 0.03);
    goal_position_exit_tolerance_ = declare_parameter(
      "goal_position_exit_tolerance_m", 0.04);
    start_approach_tolerance_ = declare_parameter(
      "start_approach_tolerance_m", 0.05);
    start_approach_exit_tolerance_ = declare_parameter(
      "start_approach_exit_tolerance_m", 0.06);
    goal_heading_exit_tolerance_ = degreesToRadians(
      declare_parameter("goal_heading_exit_tolerance_deg", 3.0));
    stopped_linear_speed_ = declare_parameter("stopped_linear_speed_mps", 0.01);
    stopped_angular_speed_ = declare_parameter("stopped_angular_speed_rps", 0.02);
    goal_settle_duration_ = declare_parameter("goal_settle_duration_s", 0.30);
    limits_.max_deceleration = declare_parameter("max_linear_deceleration", 0.25);
    limits_.braking_deceleration = declare_parameter(
      "braking_profile_deceleration", 0.12);
    limits_.gravity_slip_ratio = declare_parameter("gravity_slip_ratio", 0.0);
    limits_.gravity_direction = {
      declare_parameter("gravity_down_x", 0.0),
      declare_parameter("gravity_down_y", -1.0)};

    linear_acceleration_ = declare_parameter("max_linear_acceleration", 0.20);
    angular_acceleration_ = declare_parameter("max_angular_acceleration", 0.80);

    // Time-parameterised tracking. The rated operating point is deliberately
    // the same in both modes - the profile below is planned from cruise_speed
    // and the two time_profile_* rates - so what separates them is how the
    // speed is arrived at, not how fast the robot is allowed to be. The
    // catch_up_* limits are the room the correction has to work in above that
    // rated point; they are what makes a late robot able to catch up instead
    // of only able to observe that it is late.
    const std::string tracking_mode =
      declare_parameter("tracking_mode", std::string("distance"));
    if (tracking_mode == "distance") {
      tracking_mode_ = TrackingMode::DISTANCE;
    } else if (tracking_mode == "time") {
      tracking_mode_ = TrackingMode::TIME;
    } else {
      throw std::invalid_argument("tracking_mode must be \"distance\" or \"time\".");
    }
    time_speed_lag_ = declare_parameter("time_speed_lag_s", 0.08);
    time_along_gain_ = declare_parameter("time_along_gain", 1.0);
    time_along_integral_gain_ = declare_parameter("time_along_integral_gain", 0.0);
    time_along_integral_limit_ = declare_parameter("time_along_integral_limit_m_s", 0.05);
    time_profile_acceleration_ = declare_parameter("time_profile_acceleration", 0.20);
    time_profile_deceleration_ = declare_parameter("time_profile_deceleration", 0.20);
    catch_up_max_linear_speed_ = declare_parameter("catch_up_max_linear_speed", 0.35);
    catch_up_max_linear_acceleration_ = declare_parameter(
      "catch_up_max_linear_acceleration", 0.35);
    time_axis_stretch_enabled_ = declare_parameter("time_axis_stretch_enabled", false);
    time_axis_stretch_lag_ = declare_parameter("time_axis_stretch_lag_m", 0.05);
    time_mode_final_approach_enabled_ = declare_parameter(
      "time_mode_final_approach_enabled", false);

    // Fixed per-segment costs the duration model cannot derive: how long the
    // heading takes to converge into the alignment deadband, how long the
    // speed takes to decay below the standstill threshold, and the executor
    // round trip between segments. All three default to zero, so the shipped
    // configuration predicts exactly what it predicted before they existed and
    // a measured shortfall has to be entered deliberately.
    schedule_align_converge_s_ = declare_parameter("schedule_align_converge_s", 0.0);
    schedule_goal_stop_s_ = declare_parameter("schedule_goal_stop_s", 0.0);
    schedule_handshake_s_ = declare_parameter("schedule_handshake_s", 0.0);
    wheel_separation_ = declare_parameter("wheel_separation", -1.0);
    wheel_speed_limit_ = declare_parameter("wheel_speed_limit", -1.0);
    wheel_acceleration_limit_ = declare_parameter("wheel_acceleration_limit", -1.0);
    validateParameters();

    // Everything this node measures a duration against runs on this clock, not
    // on ROS time. See control_clock.hpp: off sim time the node clock is the
    // settable system clock, and a backward step there stops the control timer
    // for the length of the step while the robot keeps moving on its last
    // command. Only message stamps stay on ROS time.
    control_clock_ = climbot_control::controlClock(this);
    const auto zero = zeroInstant();
    alignment_profile_start_ = zero;
    alignment_settle_start_ = zero;
    travel_start_ = zero;
    arc_entry_start_time_ = zero;
    goal_settle_start_ = zero;
    last_pose_received_time_ = zero;
    last_control_time_ = zero;
    task_start_time_ = zero;
    segment_start_time_ = zero;

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
        last_pose_received_time_ = controlNow();
        have_pose_ = true;
      });

    action_server_ = rclcpp_action::create_server<ExecuteCoverage>(
      this, "/coverage/execute",
      std::bind(&LineTrackerNode::handleGoal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&LineTrackerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&LineTrackerNode::handleAccepted, this, std::placeholders::_1));

    if (standalone_mode_) {
      publishReferencePath();
    }
    publishCompletion(false);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_frequency_hz_));
    timer_ = rclcpp::create_timer(this, control_clock_, period,
      std::bind(&LineTrackerNode::onTimer, this));
  }

private:
  /// Now, on the clock every duration in this node is measured against.
  rclcpp::Time controlNow() const
  {
    return control_clock_->now();
  }

  /// The "not set yet" instant, carrying the control clock's type. Subtracting
  /// two rclcpp::Time of different clock types throws, so the sentinel cannot
  /// be a fixed RCL_ROS_TIME zero once the clock is chosen at runtime.
  rclcpp::Time zeroInstant() const
  {
    return rclcpp::Time(0, 0, control_clock_->get_clock_type());
  }

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
    requirePositive("segment_timeout_s", segment_timeout_s_);
    requireFinite("turn_slip_per_degree_m", turn_slip_per_degree_);
    if (turn_slip_per_degree_ < 0.0) {
      throw std::invalid_argument("turn_slip_per_degree_m must be non-negative.");
    }
    requirePositive("parallel_scan_offset_m", parallel_scan_offset_);
    requirePositive("maximum_scan_offset_m", maximum_scan_offset_);
    requirePositive("arc_entry_finish_offset_m", arc_entry_finish_offset_);
    requirePositive("arc_entry_speed_mps", arc_entry_speed_);
    requirePositive("arc_entry_lookahead_m", arc_entry_lookahead_);
    requirePositive("arc_entry_heading_gain", arc_entry_heading_gain_);
    requirePositive("arc_entry_max_heading_deg", arc_entry_max_heading_);
    requirePositive("arc_entry_max_angular_speed", arc_entry_max_angular_);
    requirePositive("arc_entry_timeout_s", arc_entry_timeout_);
    if (arc_entry_finish_offset_ >= parallel_scan_offset_ ||
      parallel_scan_offset_ >= maximum_scan_offset_)
    {
      throw std::invalid_argument(
              "Arc finish, parallel, and maximum offsets must be strictly increasing.");
    }
    if (arc_entry_speed_ > cruise_speed_ || arc_entry_max_angular_ > limits_.max_angular) {
      throw std::invalid_argument("Arc-entry limits cannot exceed normal motion limits.");
    }
    requireFinite("motion_region_tolerance_m", motion_region_tolerance_);
    if (motion_region_tolerance_ < 0.0) {
      throw std::invalid_argument("motion_region_tolerance_m must be non-negative.");
    }
    requirePositive("max_linear_speed", limits_.max_linear);
    requirePositive("max_angular_speed", limits_.max_angular);
    requirePositive("max_heading_correction_deg", limits_.max_heading_correction);
    requirePositive("max_gravity_feedforward_deg", limits_.max_gravity_feedforward);
    requirePositive("max_cross_feedback_deg", limits_.max_cross_feedback);
    requirePositive("alignment_threshold_deg", limits_.alignment_threshold);
    requirePositive("cross_slowdown_start_m", limits_.cross_slowdown_start);
    requirePositive("cross_slowdown_full_m", limits_.cross_slowdown_full);
    requireFinite("cross_slowdown_min_scale", limits_.cross_slowdown_min_scale);
    if (limits_.cross_slowdown_full <= limits_.cross_slowdown_start ||
      limits_.cross_slowdown_min_scale <= 0.0 || limits_.cross_slowdown_min_scale > 1.0)
    {
      throw std::invalid_argument("Cross-track slowdown limits are invalid.");
    }
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
    requirePositive("start_approach_tolerance_m", start_approach_tolerance_);
    requirePositive("start_approach_exit_tolerance_m", start_approach_exit_tolerance_);
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
    if (start_approach_exit_tolerance_ <= start_approach_tolerance_ ||
      start_approach_exit_tolerance_ > maximum_scan_offset_)
    {
      throw std::invalid_argument(
              "Start-approach tolerances must be increasing and recoverable by scan entry.");
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
    requirePositive("braking_profile_deceleration", limits_.braking_deceleration);
    if (limits_.braking_deceleration >= limits_.max_deceleration) {
      throw std::invalid_argument(
              "braking_profile_deceleration must stay below max_linear_deceleration.");
    }
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
    requirePositive("time_profile_acceleration", time_profile_acceleration_);
    requirePositive("time_profile_deceleration", time_profile_deceleration_);
    requirePositive("catch_up_max_linear_speed", catch_up_max_linear_speed_);
    requirePositive("catch_up_max_linear_acceleration", catch_up_max_linear_acceleration_);
    requirePositive("time_axis_stretch_lag_m", time_axis_stretch_lag_);
    requireFinite("time_speed_lag_s", time_speed_lag_);
    if (time_speed_lag_ < 0.0) {
      throw std::invalid_argument("time_speed_lag_s must be non-negative.");
    }
    requireFinite("time_along_gain", time_along_gain_);
    requireFinite("time_along_integral_gain", time_along_integral_gain_);
    requirePositive("time_along_integral_limit_m_s", time_along_integral_limit_);
    if (time_along_gain_ < 0.0 || time_along_integral_gain_ < 0.0) {
      throw std::invalid_argument("time_along gains must be non-negative.");
    }
    if (catch_up_max_linear_speed_ < cruise_speed_) {
      throw std::invalid_argument(
              "catch_up_max_linear_speed must leave room above cruise_speed.");
    }
    requirePositive("wheel_separation", wheel_separation_);
    requirePositive("wheel_speed_limit", wheel_speed_limit_);
    requirePositive("wheel_acceleration_limit", wheel_acceleration_limit_);
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id cannot be empty.");
    }
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteCoverage::Goal> goal)
  {
    if (standalone_mode_) {
      RCLCPP_WARN(get_logger(), "Rejected coverage goal while standalone_mode is enabled.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_goal_) {
      RCLCPP_WARN(get_logger(), "Rejected coverage goal because another task is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (const auto error = climbot_control::validateCoverageTask(goal->task, frame_id_)) {
      RCLCPP_WARN(get_logger(), "Rejected invalid coverage task: %s", error->c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal)
  {
    return goal == active_goal_ ? rclcpp_action::CancelResponse::ACCEPT :
           rclcpp_action::CancelResponse::REJECT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal)
  {
    active_goal_ = goal;
    active_task_ = goal->get_goal()->task;
    completed_segments_ = 0U;
    current_segment_ = 0U;
    task_start_time_ = controlNow();
    approaching_start_ = true;
    waiting_for_start_pose_ = !have_pose_;
    segment_start_time_ = task_start_time_;
    planSegmentDurations();
    // Log before configuring: configureStartApproach() may finish the goal and
    // release active_task_, and acceptance is what this line reports anyway.
    RCLCPP_INFO(
      get_logger(), "Accepted coverage task '%s' revision %u with %zu segments.",
      active_task_->task_id.c_str(), active_task_->revision,
      active_task_->segment_types.size());
    if (!waiting_for_start_pose_) {
      configureStartApproach();
    }
  }

  climbot_control::DurationModel durationModel() const
  {
    climbot_control::DurationModel model;
    model.cruise_speed = cruise_speed_;
    // Whichever ramps the robot is actually going to run. In distance mode the
    // command steps to the cruise speed and the rate limiter shapes the ramp,
    // while braking follows the much gentler distance-to-stop curve; in time
    // mode both ramps are the profile's own. Predicting one while executing
    // the other would leave the reported schedule wrong by the difference.
    const bool time_mode = tracking_mode_ == TrackingMode::TIME;
    model.linear_acceleration = time_mode ? time_profile_acceleration_ : linear_acceleration_;
    model.braking_deceleration = time_mode ?
      time_profile_deceleration_ : limits_.braking_deceleration;
    model.max_turn_rate = max_turn_angular_speed_;
    model.turn_acceleration = max_turn_angular_acceleration_;
    model.align_settle_s = alignment_settle_duration_;
    model.align_converge_s = schedule_align_converge_s_;
    model.goal_settle_s = goal_settle_duration_;
    model.goal_stop_s = schedule_goal_stop_s_;
    model.handshake_s = schedule_handshake_s_;
    return model;
  }

  /// Estimate how long each segment takes, so progress can be weighted by
  /// duration. Weighting segments equally makes a 0.44 m transition advance the
  /// bar as far as a 4.5 m scan line, which it covers in well under half the
  /// time; measured baselines put that discrepancy at a factor of 2.4.
  void planSegmentDurations()
  {
    const auto model = durationModel();
    const auto & waypoints = active_task_->waypoints;
    segment_turn_estimates_.assign(active_task_->segment_types.size(), 0.0);
    segment_travel_estimates_.assign(active_task_->segment_types.size(), 0.0);
    total_duration_estimate_ = 0.0;
    double previous_heading = pose_.yaw;
    for (std::size_t index = 0; index < active_task_->segment_types.size(); ++index) {
      const auto & from = waypoints[index].position;
      const auto & to = waypoints[index + 1U].position;
      const double heading = std::atan2(to.y - from.y, to.x - from.x);
      const double turn = std::atan2(
        std::sin(heading - previous_heading), std::cos(heading - previous_heading));
      previous_heading = heading;
      segment_turn_estimates_[index] =
        climbot_control::estimateTurnDuration(turn, model);
      segment_travel_estimates_[index] = climbot_control::estimateTravelDuration(
        std::hypot(to.x - from.x, to.y - from.y), model);
      total_duration_estimate_ +=
        segment_turn_estimates_[index] + segment_travel_estimates_[index];
    }
    // Deliberately outside total_duration_estimate_, which is the progress
    // bar's denominator and has to keep meaning what it meant: the bar counts
    // the segments and reads zero until the first one starts. The approach to
    // the first waypoint is real time the operator waits through, though, so
    // the schedule reported alongside the bar does include it.
    const auto & first = waypoints.front().position;
    const double approach_length = std::hypot(first.x - pose_.x, first.y - pose_.y);
    start_approach_estimate_ = approach_length <= goal_position_tolerance_ ? 0.0 :
      climbot_control::estimateSegmentDuration(
      approach_length,
      std::atan2(
        std::sin(std::atan2(first.y - pose_.y, first.x - pose_.x) - pose_.yaw),
        std::cos(std::atan2(first.y - pose_.y, first.x - pose_.x) - pose_.yaw)),
      model);
  }

  /// How much of the current segment's turn is done. The alignment profile is
  /// the controller's own plan for this turn, so the bar advances at the rate
  /// the robot is actually turning instead of jumping when the turn ends.
  double alignmentFraction() const
  {
    switch (motion_state_) {
      case MotionState::WAITING_FOR_ALIGNMENT:
      case MotionState::ALIGN_BRAKE:
        return 0.0;
      case MotionState::ALIGN_PROFILE:
        return alignment_profile_.duration > 0.0 ?
               std::clamp(
          (controlNow() - alignment_profile_start_).seconds() / alignment_profile_.duration,
          0.0, 1.0) : 1.0;
      default:
        return 1.0;
    }
  }

  void configureStartApproach()
  {
    const auto & first = active_task_->waypoints.front().position;
    if (!climbot_control::pointInPolygon(
        pose_.x, pose_.y, active_task_->motion_region, motion_region_tolerance_) ||
      !climbot_control::pointInPolygon(
        first.x, first.y, active_task_->motion_region, motion_region_tolerance_))
    {
      finishGoal(
        ExecuteCoverage::Result::OUT_OF_BOUNDS,
        "Robot or first task waypoint lies outside the motion region.");
      return;
    }
    waiting_for_start_pose_ = false;
    if (std::hypot(first.x - pose_.x, first.y - pose_.y) <= goal_position_tolerance_) {
      approaching_start_ = false;
      configureSegment(0U);
      return;
    }
    climbot_control::Point2 target{first.x, first.y};
    if (active_task_->segment_types.front() ==
      climbot_interfaces::msg::CoverageTask::SEGMENT_SCAN)
    {
      // The approach is a transition into the first scan line, and it gets the
      // same treatment every other transition gets: its end is lifted by the
      // drop of the turn waiting at it, so the robot lands on the scan line
      // rather than below it.
      //
      // This replaces a runway point 0.40 m behind the first waypoint along
      // the scan direction. That entered along the line so the turn happened
      // outside it, which worked, but it was a second mechanism for one
      // problem and it made the robot drive past its own first waypoint and
      // come back whenever it approached from the far side - measured at about
      // 0.8 m and a 180 degree turn of pure detour. Reserving the drop needs
      // no detour at all, and unlike the runway it also covers the case where
      // there was no room for one.
      const auto & second = active_task_->waypoints[1U].position;
      const double approach_yaw = std::atan2(first.y - pose_.y, first.x - pose_.x);
      const double scan_yaw = std::atan2(second.y - first.y, second.x - first.x);
      const double gravity_norm = std::hypot(
        limits_.gravity_direction.x, limits_.gravity_direction.y);
      const double reserve = climbot_control::reservedTurnDrop(
        {pose_.x, pose_.y}, {first.x, first.y}, approach_yaw, scan_yaw,
        turn_slip_per_degree_, limits_);
      if (gravity_norm > 1e-9 && reserve > 0.0) {
        // Shorten rather than abandon: a partial lift still removes most of
        // the offset, and the residual is what firstScanEntryFits judges.
        constexpr int kLiftAttempts = 8;
        for (int attempt = 0; attempt < kLiftAttempts; ++attempt) {
          const double lift = reserve *
            (1.0 - static_cast<double>(attempt) / kLiftAttempts);
          const climbot_control::Point2 lifted{
            first.x - limits_.gravity_direction.x / gravity_norm * lift,
            first.y - limits_.gravity_direction.y / gravity_norm * lift};
          if (climbot_control::pointInPolygon(
              lifted.x, lifted.y, active_task_->motion_region,
              motion_region_tolerance_))
          {
            target = lifted;
            if (attempt > 0) {
              RCLCPP_INFO(
                get_logger(),
                "Shortened the first-scan drop reserve to %.1f mm to stay in bounds.",
                lift * 1000.0);
            }
            break;
          }
        }
      }
      if (!firstScanEntryFits(first, second, target)) {
        return;
      }
    }
    configureApproachLine(target);
  }

  /// Check that entering the first scan line without a runway still fits the
  /// post-turn offset budget, and fail here rather than after driving there.
  /// Only the component normal to the scan line matters: turn slip is along
  /// gravity, so it shifts a horizontal scan line but runs along a vertical
  /// one. Returns false and finishes the goal when the budget is exceeded.
  /// Check the offset the first scan line will be left with after the entry
  /// leg's end has been lifted. Only the part of the drop that was not
  /// reserved can still displace the line, and only its component across that
  /// line displaces it at all.
  bool firstScanEntryFits(
    const geometry_msgs::msg::Point & first, const geometry_msgs::msg::Point & second,
    const climbot_control::Point2 & lifted_target)
  {
    const double approach_heading = std::atan2(
      lifted_target.y - pose_.y, lifted_target.x - pose_.x);
    const double scan_heading = std::atan2(second.y - first.y, second.x - first.x);
    const double turn = std::abs(
      std::atan2(
        std::sin(scan_heading - approach_heading),
        std::cos(scan_heading - approach_heading)));
    const double drop = turn_slip_per_degree_ * turn * 180.0 / std::acos(-1.0);
    const double gravity_norm = std::hypot(
      limits_.gravity_direction.x, limits_.gravity_direction.y);
    const double reserved = std::hypot(
      lifted_target.x - first.x, lifted_target.y - first.y);
    const double residual = std::max(0.0, drop - reserved);
    const double normal_share = gravity_norm <= 1e-9 ? 0.0 :
      std::abs(
      -std::sin(scan_heading) * limits_.gravity_direction.x +
      std::cos(scan_heading) * limits_.gravity_direction.y) / gravity_norm;
    const double budget = start_approach_tolerance_ + residual * normal_share;
    if (budget <= maximum_scan_offset_) {
      return true;
    }
    std::ostringstream reason;
    reason << "Entering the first scan line would leave up to " <<
      budget * 1000.0 << " mm of normal offset, beyond the " <<
      maximum_scan_offset_ * 1000.0 << " mm the scan entry can recover.";
    finishGoal(ExecuteCoverage::Result::TRACKING_FAILED, reason.str());
    return false;
  }

  /// Restart an approach leg from where the alignment turn actually left the
  /// robot. The leg is captured before that turn, so the turn slip appears as
  /// an initial cross-track error; on a short leg the cross-track term
  /// saturates at max_heading_correction and still cannot work it off, which
  /// is what puts the first scan line off nominal. Scan and transition
  /// segments get the equivalent treatment from prepareDynamicReference().
  /// Plans the speed curve the time-parameterised mode drives from, at the
  /// instant the robot starts driving the segment rather than when it was
  /// handed one. What is left to travel is measured here and not taken from
  /// the nominal length: the alignment that just finished moved the robot, and
  /// lockParallelScanLine may have replaced the segment outright while it was
  /// turning. Both the ordinary path and the one out of an arc entry come
  /// through ALIGN_SETTLE, so this is the only place that has to plan.
  void beginTravel(const rclcpp::Time & current_time)
  {
    travel_start_ = current_time;
    travel_stretch_credit_ = 0.0;
    time_along_integral_ = 0.0;
    schedule_lag_ = 0.0;
    const double dx = end_.x - start_.x;
    const double dy = end_.y - start_.y;
    const double length = std::hypot(dx, dy);
    if (length <= 1e-9) {
      travel_profile_ = {};
      travel_start_along_ = 0.0;
      return;
    }
    travel_start_along_ =
      ((pose_.x - start_.x) * dx + (pose_.y - start_.y) * dy) / length;
    travel_profile_ = climbot_control::planTravel(
      length - travel_start_along_, cruise_speed_,
      time_profile_acceleration_, time_profile_deceleration_);
    if (tracking_mode_ == TrackingMode::TIME) {
      RCLCPP_INFO(
        get_logger(), "Segment %zu scheduled over %.2f s for %.3f m remaining.",
        current_segment_, travel_profile_.duration, travel_profile_.distance);
    }
  }

  /// The commanded speed in time mode: the profile's own speed as feedforward,
  /// corrected by how far behind the schedule the robot actually is. The
  /// guards trackLine puts on a speed still apply, because they are about
  /// whether the robot is on the line at all rather than about how the speed
  /// was chosen.
  double timeReferenceSpeed(
    const rclcpp::Time & current_time, double dt, const climbot_control::Command & command)
  {
    const double elapsed = (current_time - travel_start_).seconds() - travel_stretch_credit_;
    const auto sample = climbot_control::sampleTravel(travel_profile_, elapsed);
    schedule_lag_ = sample.distance - (command.along - travel_start_along_);

    // Holding the reference clock while the robot is far behind turns an
    // unrecoverable schedule into a recoverable one, at the cost of no longer
    // finishing on time. Off by default: the point of the first pass is to
    // find out how far behind it actually gets.
    if (time_axis_stretch_enabled_ && schedule_lag_ > time_axis_stretch_lag_) {
      travel_stretch_credit_ += dt;
    }

    double integral_term = 0.0;
    if (time_along_integral_gain_ > 0.0) {
      const bool at_ceiling = previous_command_.linear >= catch_up_max_linear_speed_ - 1e-9;
      const bool at_floor = previous_command_.linear <= 1e-9;
      const bool would_wind_up = (at_ceiling && schedule_lag_ > 0.0) ||
        (at_floor && schedule_lag_ < 0.0);
      if (!would_wind_up) {
        const double bound = time_along_integral_limit_ / time_along_integral_gain_;
        time_along_integral_ = std::clamp(
          time_along_integral_ + schedule_lag_ * dt, -bound, bound);
      }
      integral_term = time_along_integral_gain_ * time_along_integral_;
    }

    // Acceleration feedforward. The plant reaches a commanded speed only after
    // a lag, so commanding the curve's speed alone leaves it behind by that lag
    // times the acceleration for the whole ramp - measured at 16 mm per
    // segment, which is the entire startup shortfall. Telling it where the
    // speed is going removes that before the proportional term ever sees it,
    // and a gain cannot do the same job: it can only react once the error
    // exists, and paying it back later is what overshoots the far end.
    const double raw = sample.speed + time_speed_lag_ * sample.acceleration +
      time_along_gain_ * schedule_lag_ + integral_term;
    return climbot_control::guardSpeed(
      std::clamp(raw, 0.0, catch_up_max_linear_speed_),
      command.cross, command.heading_error, limits_);
  }

  double linearAccelerationLimit() const
  {
    return tracking_mode_ == TrackingMode::TIME ?
           catch_up_max_linear_acceleration_ : linear_acceleration_;
  }

  double linearDecelerationLimit() const
  {
    return tracking_mode_ == TrackingMode::TIME ?
           catch_up_max_linear_acceleration_ : limits_.max_deceleration;
  }

  void reanchorStartApproach()
  {
    if (!approaching_start_ || start_approach_reanchored_) {
      return;
    }
    start_approach_reanchored_ = true;
    if (std::hypot(end_.x - pose_.x, end_.y - pose_.y) <=
      goal_position_exit_tolerance_)
    {
      return;
    }
    const double moved = std::hypot(pose_.x - start_.x, pose_.y - start_.y);
    start_ = {pose_.x, pose_.y};
    publishReferencePath();
    RCLCPP_INFO(
      get_logger(), "Re-anchored the start approach %.1f mm from its captured origin.",
      moved * 1000.0);
  }

  void configureApproachLine(const climbot_control::Point2 & target)
  {
    start_ = {pose_.x, pose_.y};
    start_approach_reanchored_ = false;
    end_ = target;
    motion_state_ = MotionState::WAITING_FOR_ALIGNMENT;
    previous_command_ = {};
    cross_integral_ = 0.0;
    alignment_settle_start_ = zeroInstant();
    goal_settle_start_ = zeroInstant();
    segment_start_time_ = controlNow();
    alignment_origin_ = start_;
    arc_entry_active_ = false;
    reference_prepared_ = true;
    oscillation_monitor_->reset();
    oscillation_warning_emitted_ = false;
    publishCompletion(false);
    publishReferencePath();
    RCLCPP_INFO(
      get_logger(), "Entering first task waypoint from %.3f m away.",
      std::hypot(end_.x - start_.x, end_.y - start_.y));
  }

  void configureSegment(std::size_t index)
  {
    const auto & first = active_task_->waypoints[index].position;
    const auto & second = active_task_->waypoints[index + 1U].position;
    start_ = {first.x, first.y};
    end_ = {second.x, second.y};
    motion_state_ = MotionState::WAITING_FOR_ALIGNMENT;
    previous_command_ = {};
    cross_integral_ = 0.0;
    alignment_settle_start_ = zeroInstant();
    goal_settle_start_ = zeroInstant();
    segment_start_time_ = controlNow();
    alignment_origin_ = {pose_.x, pose_.y};
    arc_entry_active_ = false;
    oscillation_monitor_->reset();
    oscillation_warning_emitted_ = false;
    const auto segment_type = active_task_->segment_types[index];
    const bool follows_transition = index > 0U &&
      active_task_->segment_types[index - 1U] ==
      climbot_interfaces::msg::CoverageTask::SEGMENT_TRANSITION;
    const bool needs_scan_entry = segment_type ==
      climbot_interfaces::msg::CoverageTask::SEGMENT_SCAN &&
      (index == 0U || follows_transition);
    reference_prepared_ = segment_type !=
      climbot_interfaces::msg::CoverageTask::SEGMENT_TRANSITION && !needs_scan_entry;
    publishCompletion(false);
    publishReferencePath();
  }

  bool lockParallelScanLine(double cross, double along)
  {
    const auto frozen = climbot_control::parallelScanSegment(
      nominal_scan_start_, nominal_scan_end_, cross, along, final_approach_distance_);
    if (!frozen.has_value()) {
      finishGoal(
        ExecuteCoverage::Result::TRACKING_FAILED,
        "Parallel scan line has insufficient forward length remaining.");
      return false;
    }
    start_ = frozen->start;
    end_ = frozen->end;
    if (!climbot_control::pointInPolygon(
        end_.x, end_.y, active_task_->motion_region, 1e-6))
    {
      finishGoal(
        ExecuteCoverage::Result::OUT_OF_BOUNDS,
        "Parallel scan endpoint lies outside the motion region.");
      return false;
    }
    arc_entry_active_ = false;
    reference_prepared_ = true;
    publishReferencePath();
    RCLCPP_INFO(
      get_logger(), "Locked a parallel scan line %.1f mm from nominal.",
      cross * 1000.0);
    return true;
  }

  bool preparePostTurnScan()
  {
    nominal_scan_start_ = start_;
    nominal_scan_end_ = end_;
    const double dx = end_.x - start_.x;
    const double dy = end_.y - start_.y;
    const double length = std::hypot(dx, dy);
    if (length <= 1e-9) {
      finishGoal(
        ExecuteCoverage::Result::TRACKING_FAILED,
        "Nominal scan line has no length.");
      return false;
    }
    const double tx = dx / length;
    const double ty = dy / length;
    const double along = (pose_.x - start_.x) * tx + (pose_.y - start_.y) * ty;
    const double cross = -(pose_.x - start_.x) * ty + (pose_.y - start_.y) * tx;
    if (std::abs(cross) <= parallel_scan_offset_) {
      return lockParallelScanLine(cross, along);
    }
    if (std::abs(cross) > maximum_scan_offset_) {
      finishGoal(
        ExecuteCoverage::Result::TRACKING_FAILED,
        "Post-turn scan offset exceeds the maximum recoverable distance.");
      return false;
    }
    arc_entry_active_ = true;
    arc_entry_start_time_ = controlNow();
    reference_prepared_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Starting a single forward arc entry for %.1f mm post-turn scan offset.",
      cross * 1000.0);
    return true;
  }

  // The reservation trusts turn_slip_per_degree_m completely now that nothing
  // floors it with a measurement. That is only safe if the constant still
  // describes this wall, so every alignment is compared against it and a
  // sustained disagreement is reported rather than silently absorbed.
  void warnIfTurnSlipLooksStale(double observed_turn_drop)
  {
    const double degrees = alignment_turn_ * 180.0 / std::acos(-1.0);
    if (degrees < 10.0) {
      return;
    }
    const double predicted = turn_slip_per_degree_ * degrees;
    const double tolerance = std::max(0.005, 0.5 * predicted);
    if (std::abs(observed_turn_drop - predicted) <= tolerance) {
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *control_clock_, 10000,
      "Turn of %.0f deg dropped %.1f mm but turn_slip_per_degree_m predicts "
      "%.1f mm. Re-run measure_turn_slip.py for this wall.",
      degrees, observed_turn_drop * 1000.0, predicted * 1000.0);
  }

  DynamicReferenceResult prepareDynamicReference()
  {
    using Task = climbot_interfaces::msg::CoverageTask;
    const auto segment_type = active_task_->segment_types[current_segment_];
    const double observed_turn_drop = std::max(0.0,
        (pose_.x - alignment_origin_.x) * limits_.gravity_direction.x +
        (pose_.y - alignment_origin_.y) * limits_.gravity_direction.y);
    if (segment_type == Task::SEGMENT_TRANSITION) {
      const auto dynamic = climbot_control::dynamicTransitionSegment(
        *active_task_, current_segment_, {pose_.x, pose_.y},
        turn_slip_per_degree_, limits_);
      RCLCPP_INFO(
        get_logger(),
        "Prepared dynamic transition after observing %.1f mm downward motion during alignment.",
        observed_turn_drop * 1000.0);
      warnIfTurnSlipLooksStale(observed_turn_drop);
      if (!climbot_control::pointInPolygon(
          dynamic.end.x, dynamic.end.y, active_task_->motion_region, 1e-6))
      {
        finishGoal(
          ExecuteCoverage::Result::OUT_OF_BOUNDS,
          "Dynamic transition endpoint lies outside the motion region.");
        return DynamicReferenceResult::FAILED;
      }
      start_ = dynamic.start;
      end_ = dynamic.end;
    } else if (segment_type == Task::SEGMENT_SCAN &&
      (current_segment_ == 0U ||
      active_task_->segment_types[current_segment_ - 1U] == Task::SEGMENT_TRANSITION))
    {
      return preparePostTurnScan() ? DynamicReferenceResult::READY :
             DynamicReferenceResult::FAILED;
    }

    if (std::hypot(end_.x - start_.x, end_.y - start_.y) <= 1e-9) {
      finishGoal(
        ExecuteCoverage::Result::TRACKING_FAILED,
        "Dynamic execution segment collapsed to zero length.");
      return DynamicReferenceResult::FAILED;
    }
    reference_prepared_ = true;
    motion_state_ = MotionState::WAITING_FOR_ALIGNMENT;
    alignment_settle_start_ = zeroInstant();
    cross_integral_ = 0.0;
    publishReferencePath();
    return DynamicReferenceResult::REALIGN;
  }

  void finishGoal(uint16_t code, const std::string & message)
  {
    if (!active_goal_) {
      return;
    }
    geometry_msgs::msg::Twist stop;
    command_publisher_->publish(stop);
    previous_command_ = {};
    auto result = std::make_shared<ExecuteCoverage::Result>();
    result->result_code = code;
    result->message = message;
    result->completed_segments = completed_segments_;
    result->elapsed_time_s = std::max(0.0, (controlNow() - task_start_time_).seconds());
    if (code == ExecuteCoverage::Result::SUCCESS) {
      active_goal_->succeed(result);
    } else if (code == ExecuteCoverage::Result::CANCELED) {
      active_goal_->canceled(result);
    } else {
      active_goal_->abort(result);
    }
    RCLCPP_INFO(get_logger(), "Coverage execution stopped: %s", message.c_str());
    active_goal_.reset();
    active_task_.reset();
    motion_state_ = MotionState::WAITING_FOR_ALIGNMENT;
  }

  uint8_t feedbackState() const
  {
    if (approaching_start_ || waiting_for_start_pose_) {
      return ExecuteCoverage::Feedback::APPROACH_START;
    }
    switch (motion_state_) {
      case MotionState::WAITING_FOR_ALIGNMENT:
      case MotionState::ALIGN_BRAKE:
      case MotionState::ALIGN_PROFILE:
      case MotionState::ARC_ENTRY:
        return ExecuteCoverage::Feedback::ALIGN;
      case MotionState::ALIGN_SETTLE:
        return ExecuteCoverage::Feedback::TURN_SETTLE;
      case MotionState::TRACK_LINE:
        return ExecuteCoverage::Feedback::TRACK_LINE;
      case MotionState::FINAL_APPROACH:
        return ExecuteCoverage::Feedback::FINAL_APPROACH;
      case MotionState::SEGMENT_COMPLETE:
        return ExecuteCoverage::Feedback::STOPPED;
    }
    return ExecuteCoverage::Feedback::WAITING;
  }

  /// Fraction of the task's estimated duration already spent. The start
  /// approach drives along its own line, which is not a task segment, so
  /// callers exclude it: measuring it the same way made progress climb to a
  /// whole segment's worth over each approach leg and then drop back to zero.
  float taskProgress(const climbot_control::Command & command)
  {
    if (!(total_duration_estimate_ > 0.0) ||
      current_segment_ >= segment_turn_estimates_.size())
    {
      return 0.0F;
    }
    double spent = 0.0;
    for (std::size_t index = 0; index < completed_segments_; ++index) {
      spent += segment_turn_estimates_[index] + segment_travel_estimates_[index];
    }
    const double length = std::hypot(end_.x - start_.x, end_.y - start_.y);
    const double travelled = length > 0.0 ?
      std::clamp(command.along / length, 0.0, 1.0) : 0.0;
    spent += segment_turn_estimates_[current_segment_] * alignmentFraction() +
      segment_travel_estimates_[current_segment_] * travelled;
    return static_cast<float>(std::clamp(spent / total_duration_estimate_, 0.0, 1.0));
  }

  /// The lag expressed in seconds, at the rated cruise speed. The reference
  /// speed itself would be the exact divisor but it is zero at both ends of
  /// every segment, where the lag is most worth reporting; a fixed divisor
  /// keeps the number monotonic in the lag and comparable between segments.
  double scheduleLagSeconds() const
  {
    if (tracking_mode_ != TrackingMode::TIME || !(cruise_speed_ > 0.0)) {
      return 0.0;
    }
    return schedule_lag_ / cruise_speed_;
  }

  /// What is left, carrying the lag. The progress fraction is already the
  /// share of planned time spent, so the remaining segments come straight from
  /// it rather than from a second traversal that could disagree with the bar.
  double estimatedRemaining(float progress) const
  {
    const double segments = total_duration_estimate_ *
      std::clamp(1.0 - static_cast<double>(progress), 0.0, 1.0);
    const double approach = approaching_start_ || waiting_for_start_pose_ ?
      start_approach_estimate_ : 0.0;
    return std::max(0.0, segments + approach + scheduleLagSeconds());
  }

  void publishFeedback(const climbot_control::Command & command)
  {
    if (!active_goal_) {
      return;
    }
    const bool before_first_segment = approaching_start_ || waiting_for_start_pose_;
    auto feedback = std::make_shared<ExecuteCoverage::Feedback>();
    feedback->state = feedbackState();
    feedback->current_segment = before_first_segment ?
      -1 : static_cast<int32_t>(current_segment_);
    feedback->segment_type = before_first_segment ?
      0U : active_task_->segment_types[current_segment_];
    feedback->along_track_error = command.along;
    feedback->cross_track_error = command.cross;
    feedback->heading_error = command.heading_error;
    feedback->remaining_distance = command.remaining;
    feedback->progress = before_first_segment ? 0.0F : taskProgress(command);
    feedback->planned_total_s = total_duration_estimate_ + start_approach_estimate_;
    feedback->schedule_lag_s = scheduleLagSeconds();
    feedback->estimated_remaining_s = estimatedRemaining(feedback->progress);
    active_goal_->publish_feedback(feedback);
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
    const auto current_time = controlNow();
    if (!standalone_mode_) {
      if (!active_goal_) {
        previous_command_ = {};
        geometry_msgs::msg::Twist stop;
        command_publisher_->publish(stop);
        last_control_time_ = current_time;
        return;
      }
      if (active_goal_->is_canceling()) {
        finishGoal(ExecuteCoverage::Result::CANCELED, "Coverage task canceled.");
        return;
      }
      if ((current_time - segment_start_time_).seconds() > segment_timeout_s_) {
        finishGoal(ExecuteCoverage::Result::CONTROL_TIMEOUT, "Segment execution timed out.");
        return;
      }
    }
    if (!have_pose_ || current_time < last_pose_received_time_ ||
      (current_time - last_pose_received_time_).seconds() > odometry_timeout_s_)
    {
      previous_command_ = {};
      cross_integral_ = 0.0;
      if (motion_state_ != MotionState::SEGMENT_COMPLETE) {
        motion_state_ = MotionState::WAITING_FOR_ALIGNMENT;
      }
      alignment_settle_start_ = zeroInstant();
      goal_settle_start_ = zeroInstant();
      last_control_time_ = current_time;
      geometry_msgs::msg::Twist stop;
      command_publisher_->publish(stop);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *control_clock_, 2000,
        "Filtered odometry is unavailable or stale; stopping.");
      if (!standalone_mode_) {
        if (!have_pose_ &&
          (current_time - task_start_time_).seconds() <= odometry_timeout_s_)
        {
          publishFeedback({});
          return;
        }
        finishGoal(
          ExecuteCoverage::Result::LOCALIZATION_TIMEOUT,
          "Filtered odometry is unavailable or stale.");
      }
      return;
    }
    if (!standalone_mode_ && !climbot_control::pointInPolygon(
        pose_.x, pose_.y, active_task_->motion_region, motion_region_tolerance_))
    {
      finishGoal(
        ExecuteCoverage::Result::OUT_OF_BOUNDS,
        "Fused robot position left the task motion region.");
      return;
    }
    if (!standalone_mode_ && waiting_for_start_pose_) {
      configureStartApproach();
      if (!active_goal_) {
        return;
      }
    }
    const double dt = last_control_time_.nanoseconds() == 0 ?
      1.0 / control_frequency_hz_ : (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;
    if (dt <= 0.0) {
      return;
    }

    if (!standalone_mode_ && motion_state_ == MotionState::SEGMENT_COMPLETE) {
      if (approaching_start_) {
        approaching_start_ = false;
        configureSegment(0U);
        return;
      }
      ++completed_segments_;
      ++current_segment_;
      if (current_segment_ >= active_task_->segment_types.size()) {
        finishGoal(ExecuteCoverage::Result::SUCCESS, "Coverage task completed.");
        return;
      }
      configureSegment(current_segment_);
    }

    if (motion_state_ == MotionState::ARC_ENTRY) {
      const double nominal_dx = nominal_scan_end_.x - nominal_scan_start_.x;
      const double nominal_dy = nominal_scan_end_.y - nominal_scan_start_.y;
      const double nominal_length = std::hypot(nominal_dx, nominal_dy);
      const climbot_control::Point2 nominal_normal{
        -nominal_dy / nominal_length, nominal_dx / nominal_length};
      const double gravity_normal =
        limits_.gravity_direction.x * nominal_normal.x +
        limits_.gravity_direction.y * nominal_normal.y;
      const double gravity_feedforward = std::clamp(
        -std::atan(limits_.gravity_slip_ratio * gravity_normal),
        -limits_.max_gravity_feedforward, limits_.max_gravity_feedforward);
      const auto arc_command = climbot_control::followArcEntry(
        nominal_scan_start_, nominal_scan_end_, pose_, arc_entry_speed_,
        arc_entry_lookahead_, arc_entry_heading_gain_, arc_entry_max_heading_,
        arc_entry_max_angular_, gravity_feedforward);
      if (std::abs(arc_command.cross) <= arc_entry_finish_offset_) {
        // The line is frozen here and an in-place alignment follows, so that
        // turn's drop has to be in the offset or the robot ends up below the
        // line it just chose. Measured before this: the arc left the robot
        // 4.2 deg off the line it was about to freeze, and the alignment then
        // dropped it 2.1 mm.
        //
        // Not solved with a heading condition on arc_command.heading_error:
        // that error is measured against the arc's own moving target, which
        // carries the same atan2(-cross, lookahead) term, so it is small by
        // construction and says nothing about the turn still to come. The
        // angle that matters is against the line about to be frozen.
        const double line_yaw = std::atan2(
          nominal_scan_end_.y - nominal_scan_start_.y,
          nominal_scan_end_.x - nominal_scan_start_.x);
        const double remaining_turn = std::abs(
          climbot_control::wrapAngle(
            line_yaw + gravity_feedforward - pose_.yaw)) * 180.0 / std::acos(-1.0);
        const double drop = turn_slip_per_degree_ * remaining_turn;
        // Only the part across the line moves it; along it the drop merely
        // starts the scan slightly further on.
        const double gravity_norm = std::hypot(
          limits_.gravity_direction.x, limits_.gravity_direction.y);
        const double reserve = gravity_norm <= 1e-9 ? 0.0 :
          drop * (limits_.gravity_direction.x * nominal_normal.x +
          limits_.gravity_direction.y * nominal_normal.y) / gravity_norm;
        if (!lockParallelScanLine(arc_command.cross + reserve, arc_command.along)) {
          return;
        }
        motion_state_ = MotionState::WAITING_FOR_ALIGNMENT;
        alignment_origin_ = {pose_.x, pose_.y};
        alignment_settle_start_ = zeroInstant();
        cross_integral_ = 0.0;
        publishFeedback(arc_command);
        return;
      }
      if ((current_time - arc_entry_start_time_).seconds() > arc_entry_timeout_ ||
        arc_command.remaining <= final_approach_distance_)
      {
        finishGoal(
          ExecuteCoverage::Result::TRACKING_FAILED,
          "Forward arc entry did not converge before its safety limit.");
        return;
      }
      limitAndPublish(arc_command, dt, angular_acceleration_);
      publishFeedback(arc_command);
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
      publishFeedback(desired);
      return;
    }
    if (std::abs(desired.heading_error) >= alignment_reentry_threshold_) {
      cross_integral_ = 0.0;
      motion_state_ = MotionState::ALIGN_BRAKE;
      updateAlignment(current_time, dt, desired);
      publishFeedback(desired);
      return;
    }
    if (motion_state_ == MotionState::TRACK_LINE &&
      desired.remaining <= final_approach_distance_)
    {
      motion_state_ = MotionState::FINAL_APPROACH;
    }
    if (motion_state_ == MotionState::FINAL_APPROACH) {
      if (updateGoalCompletion(current_time, desired)) {
        limitAndPublish({}, dt, angular_acceleration_);
        publishFeedback(desired);
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
    // After the cross-track integral update, which rebuilds the whole command:
    // clamping before it left final_approach_speed_mps with no effect at all.
    //
    // Time mode replaces exactly one signal here and leaves the rest of the
    // command as trackLine built it, so an A/B against distance mode can only
    // differ through the linear speed. The final-approach fallback hands the
    // last few centimetres back to the distance-to-stop curve; it is off by
    // default so the first pass measures what the schedule alone lands.
    const bool distance_drives_linear = tracking_mode_ == TrackingMode::DISTANCE ||
      (motion_state_ == MotionState::FINAL_APPROACH && time_mode_final_approach_enabled_);
    if (distance_drives_linear) {
      if (motion_state_ == MotionState::FINAL_APPROACH) {
        desired.linear = std::min(desired.linear, final_approach_speed_);
      }
    } else {
      desired.linear = timeReferenceSpeed(current_time, dt, desired);
    }
    if (oscillation_monitor_->update(desired.cross, desired.along) &&
      !oscillation_warning_emitted_)
    {
      oscillation_warning_emitted_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Visible cross-track oscillation detected on segment %zu; continuing while recording it for trajectory acceptance.",
        current_segment_);
    }
    limitAndPublish(desired, dt, angular_acceleration_);
    publishFeedback(desired);
  }

  bool updateGoalCompletion(
    const rclcpp::Time & current_time, const climbot_control::Command & command)
  {
    const double position_error = std::hypot(end_.x - pose_.x, end_.y - pose_.y);
    const double heading_error = std::abs(command.heading_error);
    // The start approach keeps a loose tolerance on both legs: the robot turns
    // in place there and slips while stopped, so a tight ball around the goal
    // can be left as fast as it is entered. Where the first scan line ends up
    // is bounded by arc_entry_finish_offset_m instead.
    const double position_tolerance = approaching_start_ ?
      start_approach_tolerance_ : goal_position_tolerance_;
    const double position_exit_tolerance = approaching_start_ ?
      start_approach_exit_tolerance_ : goal_position_exit_tolerance_;
    const bool stopped = measured_linear_speed_ <= stopped_linear_speed_ &&
      measured_angular_speed_ <= stopped_angular_speed_;
    const bool strict_goal = position_error <= position_tolerance &&
      heading_error <= alignment_tolerance_ && stopped;
    const bool relaxed_goal = position_error <= position_exit_tolerance &&
      heading_error <= goal_heading_exit_tolerance_ && stopped;

    if (!relaxed_goal) {
      goal_settle_start_ = zeroInstant();
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
    publishCompletion(!approaching_start_);
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
            alignment_settle_start_ = zeroInstant();
          }
          return;
        }
      case MotionState::ALIGN_SETTLE:
        {
          if (std::abs(line_command.heading_error) <= alignment_tolerance_) {
            command.angular = 0.0;
          } else {
            alignment_settle_start_ = zeroInstant();
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
              if (!standalone_mode_ && !reference_prepared_) {
                const auto reference_result = prepareDynamicReference();
                if (reference_result == DynamicReferenceResult::FAILED ||
                  reference_result == DynamicReferenceResult::REALIGN)
                {
                  return;
                }
              }
              reanchorStartApproach();
              if (arc_entry_active_) {
                motion_state_ = MotionState::ARC_ENTRY;
              } else {
                beginTravel(current_time);
                motion_state_ = MotionState::TRACK_LINE;
              }
              cross_integral_ = 0.0;
            }
          }
          return;
        }
      case MotionState::TRACK_LINE:
      case MotionState::FINAL_APPROACH:
      case MotionState::ARC_ENTRY:
      case MotionState::SEGMENT_COMPLETE:
        return;
    }

    limitAndPublish(command, dt, max_turn_angular_acceleration_);
    if (std::abs(previous_command_.linear) <= 1e-4 &&
      std::abs(previous_command_.angular) <= 1e-3)
    {
      alignment_start_yaw_ = pose_.yaw;
      alignment_turn_ = std::abs(line_command.heading_error);
      alignment_profile_ = climbot_control::planTurn(
        line_command.heading_error, max_turn_angular_speed_,
        max_turn_angular_acceleration_);
      alignment_profile_start_ = current_time;
      alignment_settle_start_ = zeroInstant();
      motion_state_ = alignment_profile_.duration > 0.0 ?
        MotionState::ALIGN_PROFILE : MotionState::ALIGN_SETTLE;
    }
  }

  void limitAndPublish(
    const climbot_control::Command & desired, double dt, double angular_acceleration)
  {
    previous_command_ = climbot_control::rateLimit(
      desired, previous_command_, dt, linearAccelerationLimit(), linearDecelerationLimit(),
      angular_acceleration, wheel_separation_, wheel_speed_limit_,
      wheel_acceleration_limit_);
    geometry_msgs::msg::Twist command;
    command.linear.x = previous_command_.linear;
    command.angular.z = previous_command_.angular;
    command_publisher_->publish(command);
  }

  enum class TrackingMode
  {
    DISTANCE,
    TIME,
  };

  enum class MotionState
  {
    WAITING_FOR_ALIGNMENT,
    ALIGN_BRAKE,
    ALIGN_PROFILE,
    ALIGN_SETTLE,
    ARC_ENTRY,
    TRACK_LINE,
    FINAL_APPROACH,
    SEGMENT_COMPLETE,
  };

  bool have_pose_{false};
  TrackingMode tracking_mode_{TrackingMode::DISTANCE};
  double time_speed_lag_{0.08};
  double time_along_gain_{1.0};
  double time_along_integral_gain_{0.0};
  double time_along_integral_limit_{0.05};
  double time_along_integral_{0.0};
  double time_profile_acceleration_{0.20};
  double time_profile_deceleration_{0.20};
  double catch_up_max_linear_speed_{0.35};
  double catch_up_max_linear_acceleration_{0.35};
  bool time_axis_stretch_enabled_{false};
  double time_axis_stretch_lag_{0.05};
  bool time_mode_final_approach_enabled_{false};
  climbot_control::TravelProfile travel_profile_{};
  rclcpp::Time travel_start_;
  double travel_start_along_{0.0};
  double travel_stretch_credit_{0.0};
  double schedule_lag_{0.0};
  double cruise_speed_{0.20};
  double cross_gain_{1.0};
  double cross_integral_gain_{0.30};
  double cross_integral_limit_{0.10};
  double cross_integral_{0.0};
  double heading_gain_{2.0};
  double control_frequency_hz_{50.0};
  double odometry_timeout_s_{0.25};
  double segment_timeout_s_{120.0};
  double motion_region_tolerance_{0.02};
  double turn_slip_per_degree_{0.0005};
  double parallel_scan_offset_{0.045};
  double maximum_scan_offset_{0.12};
  double arc_entry_finish_offset_{0.012};
  double arc_entry_speed_{0.08};
  double arc_entry_lookahead_{0.20};
  double arc_entry_heading_gain_{2.0};
  double arc_entry_max_heading_{0.349065850};
  double arc_entry_max_angular_{0.25};
  double arc_entry_timeout_{15.0};
  double alignment_reentry_threshold_{0.209439510};
  double alignment_tolerance_{0.034906585};
  double alignment_settle_duration_{0.50};
  double turn_heading_gain_{2.0};
  double max_turn_angular_speed_{0.60};
  double max_turn_angular_acceleration_{1.00};
  double final_approach_distance_{0.05};
  double final_approach_speed_{0.08};
  double goal_position_tolerance_{0.03};
  double goal_position_exit_tolerance_{0.04};
  double start_approach_tolerance_{0.05};
  double start_approach_exit_tolerance_{0.06};
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
  bool standalone_mode_{true};
  bool reference_prepared_{true};
  bool arc_entry_active_{false};
  bool approaching_start_{false};
  bool start_approach_reanchored_{false};
  bool waiting_for_start_pose_{false};
  bool oscillation_warning_emitted_{false};
  uint32_t completed_segments_{0U};
  double schedule_align_converge_s_{0.0};
  double schedule_goal_stop_s_{0.0};
  double schedule_handshake_s_{0.0};
  double start_approach_estimate_{0.0};
  std::vector<double> segment_turn_estimates_;
  std::vector<double> segment_travel_estimates_;
  double total_duration_estimate_{0.0};
  std::size_t current_segment_{0U};
  climbot_control::Limits limits_;
  climbot_control::Point2 start_{};
  climbot_control::Point2 end_{};
  climbot_control::Point2 alignment_origin_{};
  climbot_control::Point2 nominal_scan_start_{};
  climbot_control::Point2 nominal_scan_end_{};
  climbot_control::Pose2 pose_{};
  climbot_control::Command previous_command_;
  MotionState motion_state_{MotionState::WAITING_FOR_ALIGNMENT};
  climbot_control::TurnProfile alignment_profile_;
  double alignment_start_yaw_{0.0};
  double alignment_turn_{0.0};
  // Instants on control_clock_, whose type is only known once the constructor
  // has read use_sim_time, so all of these are assigned there. Subtracting two
  // rclcpp::Time of different clock types throws, which is what a leftover
  // RCL_ROS_TIME default here would eventually cause.
  rclcpp::Time alignment_profile_start_;
  rclcpp::Time alignment_settle_start_;
  rclcpp::Time arc_entry_start_time_;
  rclcpp::Time goal_settle_start_;
  rclcpp::Time last_pose_received_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Time task_start_time_;
  rclcpp::Time segment_start_time_;
  std::optional<climbot_interfaces::msg::CoverageTask> active_task_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::unique_ptr<climbot_control::CrossTrackOscillationMonitor> oscillation_monitor_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp_action::Server<ExecuteCoverage>::SharedPtr action_server_;
  rclcpp::Clock::SharedPtr control_clock_;
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
