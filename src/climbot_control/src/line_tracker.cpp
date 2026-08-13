#include "climbot_control/line_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_control
{
double wrapAngle(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

std::optional<double> yawFromQuaternion(double x, double y, double z, double w) noexcept
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
    return std::nullopt;
  }
  const double norm = std::hypot(std::hypot(x, y), std::hypot(z, w));
  if (norm <= 1e-12) {
    return std::nullopt;
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

Command trackLine(
  const Point2 & start, const Point2 & end, const Pose2 & pose,
  double cruise_speed, double cross_gain, double heading_gain, const Limits & limits,
  double cross_integral_gain, double cross_integral)
{
  const double dx = end.x - start.x, dy = end.y - start.y, length = std::hypot(dx, dy);
  if (length <= 1e-9) {throw std::invalid_argument("Line segment must be non-zero.");}
  const double tx = dx / length, ty = dy / length;
  const double along = (pose.x - start.x) * tx + (pose.y - start.y) * ty;
  const double cross = -(pose.x - start.x) * ty + (pose.y - start.y) * tx;
  const Point2 normal{-ty, tx};
  const double gravity_normal = limits.gravity_direction.x * normal.x +
    limits.gravity_direction.y * normal.y;
  const double raw_gravity_feedforward = -std::atan(
    limits.gravity_slip_ratio * gravity_normal);
  const double gravity_feedforward = std::clamp(
    raw_gravity_feedforward,
    -limits.max_gravity_feedforward, limits.max_gravity_feedforward);
  const double raw_cross_feedback = -cross_gain * cross -
    cross_integral_gain * cross_integral;
  const double cross_feedback = std::clamp(
    raw_cross_feedback, -limits.max_cross_feedback, limits.max_cross_feedback);
  const double raw_heading_correction = gravity_feedforward + cross_feedback;
  const double heading_correction = std::clamp(raw_heading_correction,
    -limits.max_heading_correction, limits.max_heading_correction);
  const double target_yaw = std::atan2(ty, tx) + heading_correction;
  const double heading_error = wrapAngle(target_yaw - pose.yaw);
  const double remaining = length - along;
  const double braking_speed = std::sqrt(std::max(0.0,
      2.0 * limits.max_deceleration * std::max(0.0, remaining)));
  double linear = std::clamp(std::min(cruise_speed, braking_speed),
    0.0, limits.max_linear);
  if (std::abs(heading_error) > limits.alignment_threshold) {
    linear = 0.0;
  }
  const bool correction_saturated =
    gravity_feedforward != raw_gravity_feedforward || cross_feedback != raw_cross_feedback ||
    heading_correction != raw_heading_correction;
  return {linear, std::clamp(heading_gain * heading_error, -limits.max_angular, limits.max_angular),
    along, cross, remaining, heading_error, gravity_feedforward, cross_feedback,
    heading_correction, correction_saturated};
}

Command followArcEntry(
  const Point2 & nominal_start, const Point2 & nominal_end, const Pose2 & pose,
  double speed, double lookahead, double heading_gain,
  double max_heading_correction, double max_angular_speed)
{
  const double dx = nominal_end.x - nominal_start.x;
  const double dy = nominal_end.y - nominal_start.y;
  const double length = std::hypot(dx, dy);
  if (length <= 1e-9 || speed <= 0.0 || lookahead <= 0.0 || heading_gain <= 0.0 ||
    max_heading_correction <= 0.0 || max_angular_speed <= 0.0)
  {
    throw std::invalid_argument("Invalid arc-entry geometry or limits.");
  }
  const double tx = dx / length;
  const double ty = dy / length;
  const double along = (pose.x - nominal_start.x) * tx +
    (pose.y - nominal_start.y) * ty;
  const double cross = -(pose.x - nominal_start.x) * ty +
    (pose.y - nominal_start.y) * tx;
  const double heading_correction = std::clamp(
    std::atan2(-cross, lookahead),
    -max_heading_correction, max_heading_correction);
  const double target_yaw = std::atan2(ty, tx) + heading_correction;
  const double heading_error = wrapAngle(target_yaw - pose.yaw);
  return {
    speed,
    std::clamp(heading_gain * heading_error, -max_angular_speed, max_angular_speed),
    along, cross, length - along, heading_error, 0.0, 0.0,
    heading_correction, std::abs(heading_correction) >= max_heading_correction};
}

Command rateLimit(
  const Command & desired, const Command & previous, double dt,
  double linear_acceleration, double linear_deceleration, double angular_acceleration,
  double wheel_separation, double wheel_speed_limit, double wheel_acceleration_limit)
{
  if (dt <= 0.0 || linear_acceleration <= 0.0 || linear_deceleration <= 0.0 ||
    angular_acceleration <= 0.0 || wheel_separation <= 0.0 || wheel_speed_limit <= 0.0 ||
    wheel_acceleration_limit <= 0.0)
  {
    throw std::invalid_argument("Invalid limits.");
  }
  Command output = desired;
  const double linear_delta = desired.linear - previous.linear;
  const double linear_limit = (linear_delta >=
    0.0 ? linear_acceleration : linear_deceleration) * dt;
  output.linear = previous.linear + std::clamp(linear_delta, -linear_limit, linear_limit);
  output.angular = previous.angular + std::clamp(
    desired.angular - previous.angular, -angular_acceleration * dt, angular_acceleration * dt);

  const double previous_left = previous.linear - previous.angular * wheel_separation / 2.0;
  const double previous_right = previous.linear + previous.angular * wheel_separation / 2.0;
  double left = output.linear - output.angular * wheel_separation / 2.0;
  double right = output.linear + output.angular * wheel_separation / 2.0;
  const double acceleration_scale = std::max(1.0, std::max(
      std::abs(left - previous_left), std::abs(right - previous_right)) /
      (wheel_acceleration_limit * dt));
  left = previous_left + (left - previous_left) / acceleration_scale;
  right = previous_right + (right - previous_right) / acceleration_scale;

  const double speed_scale = std::max(1.0,
      std::max(std::abs(left), std::abs(right)) / wheel_speed_limit);
  left /= speed_scale;
  right /= speed_scale;
  output.linear = (left + right) / 2.0;
  output.angular = (right - left) / wheel_separation;
  return output;
}
}  // namespace climbot_control
