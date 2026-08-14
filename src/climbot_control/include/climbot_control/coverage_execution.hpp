#ifndef CLIMBOT_CONTROL__COVERAGE_EXECUTION_HPP_
#define CLIMBOT_CONTROL__COVERAGE_EXECUTION_HPP_

#include <optional>
#include <string>

#include "climbot_control/line_tracker.hpp"
#include "climbot_interfaces/msg/coverage_task.hpp"
#include "geometry_msgs/msg/polygon.hpp"

namespace climbot_control
{
std::optional<std::string> validateCoverageTask(
  const climbot_interfaces::msg::CoverageTask & task,
  const std::string & expected_frame);

bool pointInPolygon(
  double x, double y, const geometry_msgs::msg::Polygon & polygon,
  double tolerance = 1e-9);

struct ExecutionSegment
{
  Point2 start;
  Point2 end;
};

// Freeze a scan line parallel to its nominal line at the measured cross-track
// offset. Returns std::nullopt when too little forward scan length remains.
std::optional<ExecutionSegment> parallelScanSegment(
  const Point2 & nominal_start, const Point2 & nominal_end,
  double cross_track, double along_track, double minimum_remaining_length);

ExecutionSegment dynamicTransitionSegment(
  const climbot_interfaces::msg::CoverageTask & task, std::size_t segment_index,
  const Point2 & actual_start, double turn_slip_per_degree,
  const Point2 & gravity_down, double observed_previous_turn_drop = 0.0);

class CrossTrackOscillationMonitor
{
public:
  CrossTrackOscillationMonitor(
    double deadband, double minimum_reversal_travel, unsigned int maximum_reversals);

  bool update(double cross_track, double along_track);
  void reset() noexcept;
  unsigned int reversalCount() const noexcept {return reversal_count_;}

private:
  double deadband_;
  double minimum_reversal_travel_;
  unsigned int maximum_reversals_;
  int sign_{0};
  double last_reversal_along_{0.0};
  unsigned int reversal_count_{0};
};
}  // namespace climbot_control
#endif
