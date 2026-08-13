#ifndef CLIMBOT_CONTROL__TURN_PROFILE_HPP_
#define CLIMBOT_CONTROL__TURN_PROFILE_HPP_

namespace climbot_control
{
struct TurnProfile
{
  double sign{1.0};
  double peak_rate{0.0};
  double ramp_duration{0.0};
  double coast_duration{0.0};
  double acceleration{0.0};
  double duration{0.0};

  bool isTrapezoidal() const noexcept {return coast_duration > 0.0;}
};

struct TurnSample
{
  double angle{0.0};
  double angular_rate{0.0};
};

TurnProfile planTurn(double delta_angle, double max_rate, double acceleration);
TurnSample sampleTurn(const TurnProfile & profile, double elapsed);
}  // namespace climbot_control
#endif
