#include "climbot_control/coverage_execution.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_control
{
namespace
{
// Below this the transition is too short to infer a heading from: lifting its
// end swings the line faster than the reservation converges.
constexpr double kMinimumTransitionLength = 0.05;
constexpr int kReserveIterations = 4;

// The heading the robot actually holds on a line, which is the line's own
// direction plus the gravity feedforward that line calls for. A vertical line
// asks for none; a horizontal one asks for the full slip angle.
double heldHeading(double line_yaw, const Limits & limits)
{
  const double gravity_normal =
    limits.gravity_direction.x * -std::sin(line_yaw) +
    limits.gravity_direction.y * std::cos(line_yaw);
  const double feedforward = std::clamp(
    -std::atan(limits.gravity_slip_ratio * gravity_normal),
    -limits.max_gravity_feedforward, limits.max_gravity_feedforward);
  return line_yaw + feedforward;
}

double wrapTo(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finitePoint(const geometry_msgs::msg::Point32 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double cross(
  const geometry_msgs::msg::Point32 & first,
  const geometry_msgs::msg::Point32 & second,
  const geometry_msgs::msg::Point32 & third)
{
  return (second.x - first.x) * (third.y - first.y) -
         (second.y - first.y) * (third.x - first.x);
}

bool validConvexPolygon(const geometry_msgs::msg::Polygon & polygon)
{
  if (polygon.points.size() < 3U) {
    return false;
  }
  int orientation = 0;
  for (std::size_t index = 0; index < polygon.points.size(); ++index) {
    const auto & first = polygon.points[index];
    const auto & second = polygon.points[(index + 1U) % polygon.points.size()];
    const auto & third = polygon.points[(index + 2U) % polygon.points.size()];
    if (!finitePoint(first)) {
      return false;
    }
    const double turn = cross(first, second, third);
    if (std::abs(turn) <= 1e-9) {
      continue;
    }
    const int current_orientation = turn > 0.0 ? 1 : -1;
    if (orientation != 0 && orientation != current_orientation) {
      return false;
    }
    orientation = current_orientation;
  }
  return orientation != 0;
}
}  // namespace

bool pointInPolygon(
  double x, double y, const geometry_msgs::msg::Polygon & polygon, double tolerance)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(tolerance) ||
    tolerance < 0.0 || polygon.points.size() < 3U)
  {
    return false;
  }
  int orientation = 0;
  for (std::size_t index = 0; index < polygon.points.size(); ++index) {
    const auto & first = polygon.points[index];
    const auto & second = polygon.points[(index + 1U) % polygon.points.size()];
    const double edge_x = second.x - first.x;
    const double edge_y = second.y - first.y;
    const double edge_length = std::hypot(edge_x, edge_y);
    if (!std::isfinite(edge_length) || edge_length <= 1e-9) {
      return false;
    }
    const double side = edge_x * (y - first.y) - edge_y * (x - first.x);
    if (std::abs(side) / edge_length <= tolerance) {
      continue;
    }
    const int current_orientation = side > 0.0 ? 1 : -1;
    if (orientation != 0 && orientation != current_orientation) {
      return false;
    }
    orientation = current_orientation;
  }
  return true;
}

std::optional<ExecutionSegment> parallelScanSegment(
  const Point2 & nominal_start, const Point2 & nominal_end,
  double cross_track, double along_track, double minimum_remaining_length)
{
  if (!std::isfinite(nominal_start.x) || !std::isfinite(nominal_start.y) ||
    !std::isfinite(nominal_end.x) || !std::isfinite(nominal_end.y) ||
    !std::isfinite(cross_track) || !std::isfinite(along_track) ||
    !std::isfinite(minimum_remaining_length) || minimum_remaining_length < 0.0)
  {
    throw std::invalid_argument("Parallel scan inputs must be finite and valid.");
  }
  const double dx = nominal_end.x - nominal_start.x;
  const double dy = nominal_end.y - nominal_start.y;
  const double length = std::hypot(dx, dy);
  if (length <= 1e-9) {
    throw std::invalid_argument("Nominal scan line must be non-zero.");
  }
  const double forward_along = std::max(0.0, along_track);
  if (length - forward_along <= minimum_remaining_length) {
    return std::nullopt;
  }
  const double tx = dx / length;
  const double ty = dy / length;
  const Point2 normal{-ty, tx};
  return ExecutionSegment{
    {nominal_start.x + cross_track * normal.x + forward_along * tx,
      nominal_start.y + cross_track * normal.y + forward_along * ty},
    {nominal_end.x + cross_track * normal.x,
      nominal_end.y + cross_track * normal.y}};
}

double reservedTurnDrop(
  const Point2 & actual_start, const Point2 & nominal_end,
  double nominal_leg_yaw, double next_line_yaw,
  double turn_slip_per_degree, const Limits & limits)
{
  const double gravity_norm = std::hypot(
    limits.gravity_direction.x, limits.gravity_direction.y);
  if (!std::isfinite(turn_slip_per_degree) || turn_slip_per_degree < 0.0 ||
    gravity_norm <= 1e-9)
  {
    return 0.0;
  }
  const double up_x = -limits.gravity_direction.x / gravity_norm;
  const double up_y = -limits.gravity_direction.y / gravity_norm;
  // The turn ends with the robot holding the next line's compensated heading,
  // not the line's own direction.
  const double held_after = heldHeading(next_line_yaw, limits);
  const double maximum = turn_slip_per_degree * 180.0;
  double reserve = 0.0;
  for (int iteration = 0; iteration < kReserveIterations; ++iteration) {
    const double end_x = nominal_end.x + up_x * reserve;
    const double end_y = nominal_end.y + up_y * reserve;
    const double span = std::hypot(end_x - actual_start.x, end_y - actual_start.y);
    // Too short to read a heading from, so fall back to the nominal one rather
    // than amplify a millimetre of position noise into tens of degrees.
    const double driven_yaw = span >= kMinimumTransitionLength ?
      std::atan2(end_y - actual_start.y, end_x - actual_start.x) :
      nominal_leg_yaw;
    const double held_before = heldHeading(driven_yaw, limits);
    const double turn_degrees =
      std::abs(wrapTo(held_after - held_before)) * 180.0 / std::acos(-1.0);
    reserve = std::clamp(turn_slip_per_degree * turn_degrees, 0.0, maximum);
  }
  return reserve;
}

ExecutionSegment dynamicTransitionSegment(
  const climbot_interfaces::msg::CoverageTask & task, std::size_t segment_index,
  const Point2 & actual_start, double turn_slip_per_degree,
  const Limits & limits)
{
  using Task = climbot_interfaces::msg::CoverageTask;
  if (segment_index >= task.segment_types.size() ||
    task.segment_types[segment_index] != Task::SEGMENT_TRANSITION)
  {
    throw std::invalid_argument("Dynamic reference requires a TRANSITION segment.");
  }
  if (!std::isfinite(actual_start.x) || !std::isfinite(actual_start.y) ||
    !std::isfinite(turn_slip_per_degree) || turn_slip_per_degree < 0.0 ||
    !std::isfinite(limits.gravity_direction.x) ||
    !std::isfinite(limits.gravity_direction.y) ||
    !std::isfinite(limits.gravity_slip_ratio) ||
    !std::isfinite(limits.max_gravity_feedforward))
  {
    throw std::invalid_argument("Dynamic transition inputs must be finite and valid.");
  }
  const double gravity_norm = std::hypot(
    limits.gravity_direction.x, limits.gravity_direction.y);
  if (gravity_norm <= 1e-9) {
    throw std::invalid_argument("Gravity direction must be non-zero.");
  }
  const auto & nominal_end = task.waypoints[segment_index + 1U].position;
  ExecutionSegment segment{actual_start, {nominal_end.x, nominal_end.y}};
  if (segment_index + 1U >= task.segment_types.size()) {
    return segment;
  }

  const auto & nominal_start = task.waypoints[segment_index].position;
  const auto & next_end = task.waypoints[segment_index + 2U].position;
  const double nominal_transition_yaw = std::atan2(
    nominal_end.y - nominal_start.y, nominal_end.x - nominal_start.x);
  const double next_scan_yaw = std::atan2(
    next_end.y - nominal_end.y, next_end.x - nominal_end.x);

  // Every turn onto a line gets this reservation, whatever the task calls
  // itself. Vertical sweeps were exempt on the reasoning that their drop runs
  // along the column and is therefore along-track error the tracker removes.
  // It does not: a drop at the *start* of a line does not get tracked out, it
  // shortens the line. Measured on the 3.30 x 4.50 m vertical rectangle, every
  // downward column stopped 46 mm below the region top - one turn's worth of
  // drop - while every upward column cleared it, because there the same drop
  // pushes the start backwards and merely over-scans below.
  const double reserve = reservedTurnDrop(
    actual_start, {nominal_end.x, nominal_end.y}, nominal_transition_yaw,
    next_scan_yaw, turn_slip_per_degree, limits);
  segment.end.x -= limits.gravity_direction.x / gravity_norm * reserve;
  segment.end.y -= limits.gravity_direction.y / gravity_norm * reserve;
  return segment;
}

std::optional<std::string> validateCoverageTask(
  const climbot_interfaces::msg::CoverageTask & task,
  const std::string & expected_frame)
{
  using Task = climbot_interfaces::msg::CoverageTask;
  if (task.task_id.empty()) {
    return "task_id cannot be empty";
  }
  if (task.revision == 0U) {
    return "revision must be non-zero";
  }
  if (task.header.frame_id.empty() || task.header.frame_id != expected_frame) {
    return "task frame must match the controller frame";
  }
  if (task.sweep_direction != Task::SWEEP_HORIZONTAL &&
    task.sweep_direction != Task::SWEEP_VERTICAL)
  {
    return "sweep_direction is invalid";
  }
  if (!std::isfinite(task.detection_width) || task.detection_width <= 0.0 ||
    !std::isfinite(task.detection_length) || task.detection_length <= 0.0)
  {
    return "detection dimensions must be finite and positive";
  }
  if (task.waypoints.size() < 2U ||
    task.segment_types.size() + 1U != task.waypoints.size())
  {
    return "waypoint and segment counts are inconsistent";
  }
  if (!validConvexPolygon(task.coverage_region) ||
    !validConvexPolygon(task.motion_region))
  {
    return "coverage_region and motion_region must be finite convex polygons";
  }

  for (std::size_t index = 0; index < task.waypoints.size(); ++index) {
    const auto & pose = task.waypoints[index];
    const auto & position = pose.position;
    const auto & orientation = pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      return "waypoints must contain only finite values";
    }
    const double quaternion_norm = std::hypot(
      std::hypot(orientation.x, orientation.y),
      std::hypot(orientation.z, orientation.w));
    if (quaternion_norm <= 1e-9) {
      return "waypoint quaternion cannot be zero";
    }
    if (!pointInPolygon(position.x, position.y, task.motion_region, 1e-6)) {
      return "every waypoint must lie inside motion_region";
    }
    if (index + 1U < task.waypoints.size()) {
      const auto & next = task.waypoints[index + 1U].position;
      if (std::hypot(next.x - position.x, next.y - position.y) <= 1e-9) {
        return "zero-length segments are invalid";
      }
      const auto segment_type = task.segment_types[index];
      if (segment_type != Task::SEGMENT_SCAN &&
        segment_type != Task::SEGMENT_TRANSITION &&
        segment_type != Task::SEGMENT_RETURN)
      {
        return "segment type is invalid";
      }
    }
  }
  return std::nullopt;
}

CrossTrackOscillationMonitor::CrossTrackOscillationMonitor(
  double deadband, double minimum_reversal_travel, unsigned int maximum_reversals)
: deadband_(deadband),
  minimum_reversal_travel_(minimum_reversal_travel),
  maximum_reversals_(maximum_reversals)
{
  if (!std::isfinite(deadband_) || deadband_ <= 0.0 ||
    !std::isfinite(minimum_reversal_travel_) || minimum_reversal_travel_ <= 0.0)
  {
    throw std::invalid_argument("Oscillation monitor distances must be finite and positive.");
  }
}

bool CrossTrackOscillationMonitor::update(double cross_track, double along_track)
{
  if (!std::isfinite(cross_track) || !std::isfinite(along_track)) {
    return false;
  }
  if (std::abs(cross_track) < deadband_) {
    return false;
  }
  const int current_sign = cross_track > 0.0 ? 1 : -1;
  if (sign_ == 0) {
    sign_ = current_sign;
    last_reversal_along_ = along_track;
    return false;
  }
  if (current_sign != sign_ &&
    std::abs(along_track - last_reversal_along_) >= minimum_reversal_travel_)
  {
    sign_ = current_sign;
    last_reversal_along_ = along_track;
    ++reversal_count_;
  }
  return reversal_count_ > maximum_reversals_;
}

void CrossTrackOscillationMonitor::reset() noexcept
{
  sign_ = 0;
  last_reversal_along_ = 0.0;
  reversal_count_ = 0U;
}
}  // namespace climbot_control
