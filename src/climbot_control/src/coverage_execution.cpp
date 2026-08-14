#include "climbot_control/coverage_execution.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_control
{
namespace
{
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

ExecutionSegment dynamicTransitionSegment(
  const climbot_interfaces::msg::CoverageTask & task, std::size_t segment_index,
  const Point2 & actual_start, double turn_slip_per_degree,
  const Point2 & gravity_down, double observed_previous_turn_drop)
{
  using Task = climbot_interfaces::msg::CoverageTask;
  if (segment_index >= task.segment_types.size() ||
    task.segment_types[segment_index] != Task::SEGMENT_TRANSITION)
  {
    throw std::invalid_argument("Dynamic reference requires a TRANSITION segment.");
  }
  if (!std::isfinite(actual_start.x) || !std::isfinite(actual_start.y) ||
    !std::isfinite(turn_slip_per_degree) || turn_slip_per_degree < 0.0 ||
    !std::isfinite(gravity_down.x) || !std::isfinite(gravity_down.y) ||
    !std::isfinite(observed_previous_turn_drop) || observed_previous_turn_drop < 0.0)
  {
    throw std::invalid_argument("Dynamic transition inputs must be finite and valid.");
  }
  const double gravity_norm = std::hypot(gravity_down.x, gravity_down.y);
  if (gravity_norm <= 1e-9) {
    throw std::invalid_argument("Gravity direction must be non-zero.");
  }
  const auto & nominal_end = task.waypoints[segment_index + 1U].position;
  ExecutionSegment segment{actual_start, {nominal_end.x, nominal_end.y}};
  if (task.sweep_direction != Task::SWEEP_HORIZONTAL ||
    segment_index + 1U >= task.segment_types.size())
  {
    return segment;
  }

  const auto & nominal_start = task.waypoints[segment_index].position;
  const auto & next_end = task.waypoints[segment_index + 2U].position;
  const double transition_yaw = std::atan2(
    nominal_end.y - nominal_start.y, nominal_end.x - nominal_start.x);
  const double next_scan_yaw = std::atan2(
    next_end.y - nominal_end.y, next_end.x - nominal_end.x);
  const double turn_degrees = std::abs(std::atan2(
      std::sin(next_scan_yaw - transition_yaw),
      std::cos(next_scan_yaw - transition_yaw))) * 180.0 / std::acos(-1.0);
  const double predicted_drop = std::max(
    turn_slip_per_degree * turn_degrees, observed_previous_turn_drop);
  segment.end.x -= gravity_down.x / gravity_norm * predicted_drop;
  segment.end.y -= gravity_down.y / gravity_norm * predicted_drop;
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
    return true;
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
