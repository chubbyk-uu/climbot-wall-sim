#include "climbot_control/segment_duration.hpp"

#include <algorithm>
#include <cmath>

#include "climbot_control/turn_profile.hpp"

namespace climbot_control
{

double estimateTurnDuration(double turn_angle, const DurationModel & model)
{
  if (!std::isfinite(turn_angle)) {
    return model.settle_duration;
  }
  const auto profile = planTurn(turn_angle, model.max_turn_rate, model.turn_acceleration);
  return profile.duration + model.settle_duration;
}

double estimateTravelDuration(double length, const DurationModel & model)
{
  if (!std::isfinite(length) || length <= 0.0 ||
    !(model.cruise_speed > 0.0) || !(model.linear_acceleration > 0.0) ||
    !(model.braking_deceleration > 0.0))
  {
    return 0.0;
  }
  const double accel = model.cruise_speed / model.linear_acceleration;
  const double brake = model.cruise_speed / model.braking_deceleration;
  const double ramp_distance = 0.5 * model.cruise_speed * (accel + brake);
  if (length >= ramp_distance) {
    return accel + brake + (length - ramp_distance) / model.cruise_speed;
  }
  // Too short to reach cruise speed: it accelerates to a lower peak and brakes
  // straight back down, so solve for the peak the two ramps can share.
  const double harmonic =
    model.linear_acceleration * model.braking_deceleration /
    (model.linear_acceleration + model.braking_deceleration);
  const double peak = std::sqrt(2.0 * harmonic * length);
  return peak / model.linear_acceleration + peak / model.braking_deceleration;
}

double estimateSegmentDuration(double length, double turn_angle, const DurationModel & model)
{
  return estimateTurnDuration(turn_angle, model) + estimateTravelDuration(length, model);
}

}  // namespace climbot_control
