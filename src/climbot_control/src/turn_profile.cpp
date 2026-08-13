#include "climbot_control/turn_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_control
{
TurnProfile planTurn(double delta_angle, double max_rate, double acceleration)
{
  if (!std::isfinite(delta_angle) || !std::isfinite(max_rate) || max_rate <= 0.0 ||
    !std::isfinite(acceleration) || acceleration <= 0.0)
  {
    throw std::invalid_argument("Turn angle and limits must be finite, with positive limits.");
  }

  const double magnitude = std::abs(delta_angle);
  const double sign = delta_angle >= 0.0 ? 1.0 : -1.0;
  double peak_rate;
  double coast_duration;
  if (magnitude >= max_rate * max_rate / acceleration) {
    peak_rate = max_rate;
    coast_duration = (magnitude - max_rate * max_rate / acceleration) / max_rate;
  } else {
    peak_rate = std::sqrt(acceleration * magnitude);
    coast_duration = 0.0;
  }
  const double ramp_duration = peak_rate / acceleration;
  return {
    sign, peak_rate, ramp_duration, coast_duration, acceleration,
    2.0 * ramp_duration + coast_duration};
}

TurnSample sampleTurn(const TurnProfile & profile, double elapsed)
{
  if (!std::isfinite(elapsed)) {
    throw std::invalid_argument("Turn profile elapsed time must be finite.");
  }
  const double time = std::max(0.0, elapsed);
  const double ramp_angle = 0.5 * profile.peak_rate * profile.ramp_duration;
  double angle;
  double rate;
  if (time < profile.ramp_duration) {
    angle = 0.5 * profile.acceleration * time * time;
    rate = profile.acceleration * time;
  } else if (time < profile.ramp_duration + profile.coast_duration) {
    angle = ramp_angle + profile.peak_rate * (time - profile.ramp_duration);
    rate = profile.peak_rate;
  } else if (time < profile.duration) {
    const double tail = time - profile.ramp_duration - profile.coast_duration;
    angle = ramp_angle + profile.peak_rate * profile.coast_duration +
      profile.peak_rate * tail - 0.5 * profile.acceleration * tail * tail;
    rate = profile.peak_rate - profile.acceleration * tail;
  } else {
    angle = 2.0 * ramp_angle + profile.peak_rate * profile.coast_duration;
    rate = 0.0;
  }
  return {profile.sign * angle, profile.sign * rate};
}
}  // namespace climbot_control
