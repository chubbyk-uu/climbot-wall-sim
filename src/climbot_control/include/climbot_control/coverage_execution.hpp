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

// A turn started while the robot sits 12 to 40 degrees off vertical, in the
// direction that keeps lowering its nose, slides about 68 mm on the spot
// before it grips again - regardless of how far it goes on to turn. Measured
// on a bare wall over the whole heading circle: the effect peaks 24 degrees
// off vertical, is absent turning the other way, and is absent again once the
// robot has swept past the band, so only the heading it *starts* from matters.
// See results/turn_band.csv.
//
// Returns the signed angle to turn first - nose up, out of the band - so the
// real turn begins from a heading that grips. Zero when the turn is already
// safe. The band edges are the measured ones plus a margin.
double turnLeadOut(double start_yaw, double heading_error);

// How far to lift a leg's end, against gravity, so the turn at its far end
// lands the robot on the next line's nominal start instead of a turn-drop
// below it. Solved as a fixed point: the lift tilts the leg the robot actually
// drives, which changes the angle it turns through, which changes the lift.
// Both ends of that turn use the heading the robot holds, gravity feedforward
// included, not the lines' own directions.
double reservedTurnDrop(
  const Point2 & actual_start, const Point2 & nominal_end,
  double nominal_leg_yaw, double next_line_yaw,
  double turn_slip_per_degree, const Limits & limits);

// Lift the transition's end so the turn at its far end lands the robot on the
// next line's nominal start instead of a turn-drop below it. The drop follows
// the angle the robot actually turns through, which depends on the headings it
// actually holds - both lines' gravity feedforward included - and on the lift
// itself, so the reservation is solved as a fixed point.
ExecutionSegment dynamicTransitionSegment(
  const climbot_interfaces::msg::CoverageTask & task, std::size_t segment_index,
  const Point2 & actual_start, double turn_slip_per_degree,
  const Limits & limits);

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
