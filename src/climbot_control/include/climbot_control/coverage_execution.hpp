// Copyright 2026 jerry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
  // True only when the terminal hard-boundary fallback shortened the dynamic
  // turn reserve. A planner-generated task should never need this; keeping it
  // visible distinguishes a safety intervention from normal execution.
  bool turn_reserve_limited{false};
};

// Freeze a scan line parallel to its nominal line at the measured cross-track
// offset. Returns std::nullopt when too little forward scan length remains.
std::optional<ExecutionSegment> parallelScanSegment(
  const Point2 & nominal_start, const Point2 & nominal_end,
  double cross_track, double along_track, double minimum_remaining_length);

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
