#ifndef CLIMBOT_CONTROL__LINE_TRACKER_HPP_
#define CLIMBOT_CONTROL__LINE_TRACKER_HPP_

#include <optional>

namespace climbot_control
{
struct Point2
{
  double x;
  double y;
};

struct Pose2
{
  double x;
  double y;
  double yaw;
};

struct Limits
{
  double max_linear{0.25};
  double max_angular{0.35};
  double max_heading_correction{0.209439510};
  double max_gravity_feedforward{0.139626340};
  double max_cross_feedback{0.139626340};
  double max_deceleration{0.25};
  // The distance-to-stop profile must demand less than the rate limiter can
  // deliver. Setting both from one constant makes the command lag the profile
  // by a step it can never recover, which overshoots every segment endpoint.
  double braking_deceleration{0.12};
  double alignment_threshold{0.174532925};
  double cross_slowdown_start{0.03};
  double cross_slowdown_full{0.08};
  double cross_slowdown_min_scale{0.25};
  double gravity_slip_ratio{0.0};
  Point2 gravity_direction{0.0, -1.0};
};

struct Command
{
  double linear{0.0};
  double angular{0.0};
  double along{0.0};
  double cross{0.0};
  double remaining{0.0};
  double heading_error{0.0};
  double gravity_feedforward{0.0};
  double cross_feedback{0.0};
  double heading_correction{0.0};
  bool correction_saturated{false};
};

double wrapAngle(double angle);
std::optional<double> yawFromQuaternion(double x, double y, double z, double w) noexcept;
Command trackLine(
  const Point2 & start, const Point2 & end, const Pose2 & pose,
  double cruise_speed, double cross_gain, double heading_gain, const Limits & limits,
  double cross_integral_gain = 0.0, double cross_integral = 0.0);
Command followArcEntry(
  const Point2 & nominal_start, const Point2 & nominal_end, const Pose2 & pose,
  double speed, double lookahead, double heading_gain,
  double max_heading_correction, double max_angular_speed,
  double gravity_feedforward);
Command rateLimit(
  const Command & desired, const Command & previous, double dt,
  double linear_acceleration, double linear_deceleration, double angular_acceleration,
  double wheel_separation, double wheel_speed_limit, double wheel_acceleration_limit);
}  // namespace climbot_control
#endif
