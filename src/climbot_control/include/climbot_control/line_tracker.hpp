#ifndef CLIMBOT_CONTROL__LINE_TRACKER_HPP_
#define CLIMBOT_CONTROL__LINE_TRACKER_HPP_

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
  double max_linear{0.15};
  double max_angular{0.35};
  double max_heading_correction{0.174532925};
  double max_deceleration{0.25};
  double alignment_threshold{0.174532925};
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
};

double wrapAngle(double angle);
Command trackLine(
  const Point2 & start, const Point2 & end, const Pose2 & pose,
  double cruise_speed, double cross_gain, double heading_gain, const Limits & limits);
Command rateLimit(
  const Command & desired, const Command & previous, double dt,
  double linear_acceleration, double linear_deceleration, double angular_acceleration,
  double wheel_separation, double wheel_speed_limit, double wheel_acceleration_limit);
}  // namespace climbot_control
#endif
