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

#ifndef CLIMBOT_CONTROL__SEGMENT_DURATION_HPP_
#define CLIMBOT_CONTROL__SEGMENT_DURATION_HPP_

namespace climbot_control
{

/// Timing constants a segment's duration can be predicted from. Every field
/// already exists as a controller parameter; nothing here is tuned separately.
struct DurationModel
{
  double cruise_speed{0.20};
  double linear_acceleration{0.20};
  double braking_deceleration{0.12};
  double max_turn_rate{0.60};
  double turn_acceleration{1.00};

  /// Dead time every segment pays once, kept as the terms it is actually made
  /// of rather than as one constant. A single number could only be fitted as a
  /// whole, and the parts do not scale together: the two holds are fixed by
  /// configuration, while convergence into the alignment deadband and the
  /// decay to a standstill depend on how the segment was entered, and differ
  /// between a horizontal scan line holding a gravity feedforward and a
  /// vertical one holding none. Splitting them is what lets a measured
  /// shortfall be attributed instead of absorbed.
  ///
  /// The two holds default to the shipped configuration and the three measured
  /// terms to zero, so an uncalibrated model predicts exactly what the single
  /// constant used to.
  double align_settle_s{0.50};
  double align_converge_s{0.0};
  double goal_settle_s{0.30};
  double goal_stop_s{0.0};
  double handshake_s{0.0};

  double segmentOverhead() const
  {
    return align_settle_s + align_converge_s + goal_settle_s + goal_stop_s + handshake_s;
  }
};

/// How long the in-place turn onto a segment takes, including settle. Uses the
/// same trapezoidal profile the controller actually executes.
double estimateTurnDuration(double turn_angle, const DurationModel & model);

/// How long driving a segment of this length takes once aligned, including the
/// acceleration and braking ramps.
double estimateTravelDuration(double length, const DurationModel & model);

/// Turn plus travel. Used to weight a progress fraction by how long each
/// segment actually takes: weighting segments equally makes a short transition
/// advance a progress bar as much as a long scan line.
double estimateSegmentDuration(double length, double turn_angle, const DurationModel & model);

}  // namespace climbot_control

#endif  // CLIMBOT_CONTROL__SEGMENT_DURATION_HPP_
