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

#ifndef CLIMBOT_CONTROL__TRAVEL_PROFILE_HPP_
#define CLIMBOT_CONTROL__TRAVEL_PROFILE_HPP_

namespace climbot_control
{
/// The straight-line counterpart of TurnProfile: a trapezoidal or triangular
/// speed curve whose area is the segment length.
///
/// Acceleration and deceleration are separate because two callers need
/// different pairs from the same formula. The time-parameterised controller
/// drives the robot from this curve, so its two ramps are symmetric and sit
/// below the rate limiter that has to deliver them. estimateTravelDuration
/// predicts how long the distance-based controller takes, and that one
/// accelerates at the rate limiter's own bound while braking on a much gentler
/// distance-to-stop curve, so its two ramps differ by nearly a factor of two.
struct TravelProfile
{
  double peak_speed{0.0};
  double acceleration_duration{0.0};
  double coast_duration{0.0};
  double braking_duration{0.0};
  double acceleration{0.0};
  double deceleration{0.0};
  double distance{0.0};
  double duration{0.0};

  bool isTrapezoidal() const noexcept {return coast_duration > 0.0;}
};

struct TravelSample
{
  double distance{0.0};
  double speed{0.0};
  /// Signed rate of change of the curve at this instant: the acceleration
  /// while ramping up, zero while coasting, minus the deceleration while
  /// braking. A plant that reaches a commanded speed only after a lag needs
  /// this to be told where the speed is going, not just where it is.
  double acceleration{0.0};
};

/// Plans the curve for one straight segment. A distance that is not finite or
/// not positive yields a zero profile rather than an error: a segment can
/// legitimately have nothing left to travel, and the caller then samples a
/// standstill. Non-positive limits are a configuration error and do throw.
TravelProfile planTravel(
  double distance, double max_speed, double acceleration, double deceleration);

/// Distance covered and speed commanded at this point in the curve. Before the
/// start it reads as standstill at the origin, after the end as standstill at
/// the far end, so a control loop can sample it without bounds-checking.
TravelSample sampleTravel(const TravelProfile & profile, double elapsed);
}  // namespace climbot_control
#endif  // CLIMBOT_CONTROL__TRAVEL_PROFILE_HPP_
