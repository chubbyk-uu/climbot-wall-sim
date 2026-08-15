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
  /// Alignment settle plus goal settle: dead time every segment pays once.
  double settle_duration{0.80};
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
