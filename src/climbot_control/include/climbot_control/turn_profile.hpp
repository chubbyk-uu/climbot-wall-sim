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
