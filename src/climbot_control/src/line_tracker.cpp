#include "climbot_control/line_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_control
{
double wrapAngle(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

Command trackLine(const Point2 & start, const Point2 & end, const Pose2 & pose,
  double cruise_speed, double cross_gain, double heading_gain, const Limits & limits)
{
  const double dx = end.x - start.x, dy = end.y - start.y, length = std::hypot(dx, dy);
  if (length <= 1e-9) {throw std::invalid_argument("Line segment must be non-zero.");}
  const double tx = dx / length, ty = dy / length;
  const double along = (pose.x - start.x) * tx + (pose.y - start.y) * ty;
  const double cross = -(pose.x - start.x) * ty + (pose.y - start.y) * tx;
  const double target_yaw = std::atan2(ty, tx) - std::clamp(cross_gain * cross,
    -limits.max_heading_correction, limits.max_heading_correction);
  const double heading_error = wrapAngle(target_yaw - pose.yaw);
  const double remaining = length - along;
  const double linear = std::clamp(std::min(cruise_speed, std::max(0.0, remaining)),
    0.0, limits.max_linear);
  return {linear, std::clamp(heading_gain * heading_error, -limits.max_angular, limits.max_angular),
    along, cross, remaining, heading_error};
}

Command rateLimit(const Command & desired, const Command & previous, double dt,
  double linear_acceleration, double angular_acceleration, double wheel_separation,
  double wheel_speed_limit)
{
  if (dt <= 0.0 || linear_acceleration <= 0.0 || angular_acceleration <= 0.0 ||
    wheel_separation <= 0.0 || wheel_speed_limit <= 0.0) {throw std::invalid_argument("Invalid limits.");}
  Command output = desired;
  output.linear = previous.linear + std::clamp(desired.linear - previous.linear, -linear_acceleration * dt, linear_acceleration * dt);
  output.angular = previous.angular + std::clamp(desired.angular - previous.angular, -angular_acceleration * dt, angular_acceleration * dt);
  const double left = output.linear - output.angular * wheel_separation / 2.0;
  const double right = output.linear + output.angular * wheel_separation / 2.0;
  const double scale = std::max(1.0, std::max(std::abs(left), std::abs(right)) / wheel_speed_limit);
  output.linear /= scale; output.angular /= scale;
  return output;
}
}  // namespace climbot_control
