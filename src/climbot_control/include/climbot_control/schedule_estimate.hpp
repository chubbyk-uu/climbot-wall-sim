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

#ifndef CLIMBOT_CONTROL__SCHEDULE_ESTIMATE_HPP_
#define CLIMBOT_CONTROL__SCHEDULE_ESTIMATE_HPP_

#include <cstddef>
#include <vector>

#include "climbot_control/line_tracker.hpp"
#include "climbot_control/segment_duration.hpp"

namespace climbot_control
{

/// How long a task should take, and how much of that has been spent.
///
/// This is arithmetic over a plan, a pose and two fractions the controller
/// supplies: nothing here reads a clock, commands a wheel, or knows which of
/// the eight motion states the robot is in. Keeping it that way is the point -
/// the numbers it produces are the ones an operator schedules work from, and
/// they were previously only reachable by running a whole task in simulation.
///
/// The two fractions come from the controller because only it knows them:
/// turn_fraction is how much of the current turn its own alignment profile has
/// executed, and travel_fraction is how far along the current line it is. Both
/// are 0 to 1.
class ScheduleEstimate
{
public:
  /// Estimate every segment, and the approach to the first waypoint.
  ///
  /// waypoints are the task's nominal positions, robot is where the robot
  /// actually is, and arrival_tolerance is the distance within which the robot
  /// counts as already at the first waypoint, so no approach is planned.
  void plan(
    const std::vector<Point2> & waypoints, const Point2 & robot, double robot_yaw,
    const DurationModel & model, double arrival_tolerance);

  /// Planned time for the task's own segments, in seconds.
  ///
  /// Deliberately excludes the approach to the first waypoint: this is the
  /// progress bar's denominator, and the bar counts the task's segments. The
  /// approach is real time an operator waits through, so it is carried
  /// separately and added to the schedule rather than to the bar.
  double totalDuration() const {return total_;}

  /// Whether an approach to the first waypoint was planned at all.
  bool hasApproach() const {return approach_turn_ + approach_travel_ > 0.0;}

  /// Everything the operator waits through: the segments and the approach.
  ///
  /// This is what the schedule reports, and it is deliberately not the same
  /// number as totalDuration() - see the note there.
  double plannedTotal() const {return total_ + approach_turn_ + approach_travel_;}

  /// Forget the plan, so nothing reports the previous task's numbers.
  void clear() {*this = ScheduleEstimate{};}

  /// Share of the planned segment time spent, 0 to 1.
  double progress(
    std::size_t completed_segments, std::size_t current_segment,
    double turn_fraction, double travel_fraction) const;

  /// What is left of the drive to the first waypoint, in seconds.
  ///
  /// Kept as its turn and its drive rather than as one block, because they
  /// finish at different times: counting the leg as a single block left the
  /// countdown standing still and then jumping by the whole leg on arrival.
  double approachRemaining(double turn_fraction, double travel_remaining) const;

  /// What is left overall, in seconds, carrying the schedule lag.
  ///
  /// The remaining segments come from the progress fraction rather than from a
  /// second traversal of the plan, so this number and the bar cannot disagree.
  double remaining(double progress, double approach_remaining, double lag_s) const;

private:
  std::vector<double> segment_turns_;
  std::vector<double> segment_travels_;
  double total_{0.0};
  double approach_turn_{0.0};
  double approach_travel_{0.0};
};

}  // namespace climbot_control

#endif  // CLIMBOT_CONTROL__SCHEDULE_ESTIMATE_HPP_
