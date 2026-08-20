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

#include "climbot_control/segment_duration.hpp"

#include <algorithm>
#include <cmath>

#include "climbot_control/travel_profile.hpp"
#include "climbot_control/turn_profile.hpp"

namespace climbot_control
{

double estimateTurnDuration(double turn_angle, const DurationModel & model)
{
  if (!std::isfinite(turn_angle)) {
    return model.segmentOverhead();
  }
  const auto profile = planTurn(turn_angle, model.max_turn_rate, model.turn_acceleration);
  return profile.duration + model.segmentOverhead();
}

double estimateTravelDuration(double length, const DurationModel & model)
{
  if (!std::isfinite(length) || length <= 0.0 ||
    !(model.cruise_speed > 0.0) || !(model.linear_acceleration > 0.0) ||
    !(model.braking_deceleration > 0.0))
  {
    return 0.0;
  }
  // The same curve the time-parameterised controller drives from, only asked
  // for the pair of ramps the distance-based controller actually produces: it
  // accelerates at the rate limiter's bound and brakes on the far gentler
  // distance-to-stop curve.
  return planTravel(
    length, model.cruise_speed, model.linear_acceleration,
    model.braking_deceleration).duration;
}

double estimateSegmentDuration(double length, double turn_angle, const DurationModel & model)
{
  return estimateTurnDuration(turn_angle, model) + estimateTravelDuration(length, model);
}

}  // namespace climbot_control
