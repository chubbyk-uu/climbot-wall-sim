#include "climbot_control/travel_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_control
{
TravelProfile planTravel(
  double distance, double max_speed, double acceleration, double deceleration)
{
  if (!std::isfinite(max_speed) || max_speed <= 0.0 ||
    !std::isfinite(acceleration) || acceleration <= 0.0 ||
    !std::isfinite(deceleration) || deceleration <= 0.0)
  {
    throw std::invalid_argument("Travel limits must be finite and positive.");
  }
  if (!std::isfinite(distance) || distance <= 0.0) {
    return {};
  }

  // Distance the two ramps alone would cover if the curve did reach max_speed.
  // Written in exactly the form estimateTravelDuration used before it
  // delegated here, and summed in the same order below, so the progress bar
  // that weights segments by this duration keeps its baseline bit for bit.
  const double full_acceleration = max_speed / acceleration;
  const double full_braking = max_speed / deceleration;
  const double ramp_distance = 0.5 * max_speed * (full_acceleration + full_braking);
  double peak_speed;
  double coast_duration;
  if (distance >= ramp_distance) {
    peak_speed = max_speed;
    coast_duration = (distance - ramp_distance) / max_speed;
  } else {
    // Too short to reach max_speed: solve 0.5 * p^2 * (1/a + 1/d) = distance
    // for the peak the two ramps can share, which is the harmonic mean form.
    const double harmonic = acceleration * deceleration / (acceleration + deceleration);
    peak_speed = std::sqrt(2.0 * harmonic * distance);
    coast_duration = 0.0;
  }
  const double acceleration_duration = peak_speed / acceleration;
  const double braking_duration = peak_speed / deceleration;
  return {
    peak_speed, acceleration_duration, coast_duration, braking_duration,
    acceleration, deceleration, distance,
    acceleration_duration + braking_duration + coast_duration};
}

TravelSample sampleTravel(const TravelProfile & profile, double elapsed)
{
  if (!std::isfinite(elapsed)) {
    throw std::invalid_argument("Travel profile elapsed time must be finite.");
  }
  if (!(profile.duration > 0.0)) {
    return {profile.distance, 0.0};
  }
  const double time = std::max(0.0, elapsed);
  const double acceleration_distance =
    0.5 * profile.peak_speed * profile.acceleration_duration;
  const double coast_distance = profile.peak_speed * profile.coast_duration;
  if (time < profile.acceleration_duration) {
    return {0.5 * profile.acceleration * time * time, profile.acceleration * time};
  }
  if (time < profile.acceleration_duration + profile.coast_duration) {
    const double coasted = time - profile.acceleration_duration;
    return {
      acceleration_distance + profile.peak_speed * coasted, profile.peak_speed};
  }
  if (time < profile.duration) {
    const double tail = time - profile.acceleration_duration - profile.coast_duration;
    return {
      acceleration_distance + coast_distance + profile.peak_speed * tail -
      0.5 * profile.deceleration * tail * tail,
      profile.peak_speed - profile.deceleration * tail};
  }
  return {profile.distance, 0.0};
}
}  // namespace climbot_control
