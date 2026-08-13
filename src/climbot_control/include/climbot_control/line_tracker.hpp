#ifndef CLIMBOT_CONTROL__LINE_TRACKER_HPP_
#define CLIMBOT_CONTROL__LINE_TRACKER_HPP_

namespace climbot_control
{
struct Point2 {double x; double y;};
struct Pose2 {double x; double y; double yaw;};
struct Limits {double max_linear{0.15}; double max_angular{0.35}; double max_heading_correction{0.174532925};};
struct Command {double linear; double angular; double along; double cross; double remaining; double heading_error;};

double wrapAngle(double angle);
Command trackLine(const Point2 & start, const Point2 & end, const Pose2 & pose,
  double cruise_speed, double cross_gain, double heading_gain, const Limits & limits);
Command rateLimit(const Command & desired, const Command & previous, double dt,
  double linear_acceleration, double angular_acceleration, double wheel_separation,
  double wheel_speed_limit);
}  // namespace climbot_control
#endif
