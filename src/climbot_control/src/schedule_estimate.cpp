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

#include "climbot_control/schedule_estimate.hpp"

#include <algorithm>
#include <cmath>

namespace climbot_control
{

void ScheduleEstimate::plan(
  const std::vector<Point2> & waypoints, const Point2 & robot, double robot_yaw,
  const DurationModel & model, double arrival_tolerance)
{
  const std::size_t segments = waypoints.size() > 1U ? waypoints.size() - 1U : 0U;
  segment_turns_.assign(segments, 0.0);
  segment_travels_.assign(segments, 0.0);
  total_ = 0.0;
  approach_turn_ = 0.0;
  approach_travel_ = 0.0;
  if (segments == 0U) {
    return;
  }

  double previous_heading = robot_yaw;
  for (std::size_t index = 0; index < segments; ++index) {
    const auto & from = waypoints[index];
    const auto & to = waypoints[index + 1U];
    const double heading = std::atan2(to.y - from.y, to.x - from.x);
    segment_turns_[index] =
      estimateTurnDuration(wrapAngle(heading - previous_heading), model);
    previous_heading = heading;
    segment_travels_[index] =
      estimateTravelDuration(std::hypot(to.x - from.x, to.y - from.y), model);
    total_ += segment_turns_[index] + segment_travels_[index];
  }

  const auto & first = waypoints.front();
  const double approach_length = std::hypot(first.x - robot.x, first.y - robot.y);
  if (approach_length <= arrival_tolerance) {
    return;
  }
  const double approach_heading = std::atan2(first.y - robot.y, first.x - robot.x);
  approach_turn_ = estimateTurnDuration(wrapAngle(approach_heading - robot_yaw), model);
  approach_travel_ = estimateTravelDuration(approach_length, model);
}

double ScheduleEstimate::progress(
  std::size_t completed_segments, std::size_t current_segment,
  double turn_fraction, double travel_fraction) const
{
  if (!(total_ > 0.0) || current_segment >= segment_turns_.size()) {
    return 0.0;
  }
  double spent = 0.0;
  for (std::size_t index = 0; index < completed_segments && index < segment_turns_.size();
    ++index)
  {
    spent += segment_turns_[index] + segment_travels_[index];
  }
  spent += segment_turns_[current_segment] * std::clamp(turn_fraction, 0.0, 1.0) +
    segment_travels_[current_segment] * std::clamp(travel_fraction, 0.0, 1.0);
  return std::clamp(spent / total_, 0.0, 1.0);
}

double ScheduleEstimate::approachRemaining(
  double turn_fraction, double travel_remaining) const
{
  const double total = approach_turn_ + approach_travel_;
  if (!(total > 0.0)) {
    return total;
  }
  return approach_turn_ * (1.0 - std::clamp(turn_fraction, 0.0, 1.0)) +
         approach_travel_ * std::clamp(travel_remaining, 0.0, 1.0);
}

double ScheduleEstimate::remaining(
  double progress, double approach_remaining, double lag_s) const
{
  const double segments = total_ * std::clamp(1.0 - progress, 0.0, 1.0);
  return std::max(0.0, segments + approach_remaining + lag_s);
}

}  // namespace climbot_control
