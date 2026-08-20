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

#include "climbot_control/segment_arrival.hpp"

#include <cmath>

namespace climbot_control
{

void SegmentArrival::reset()
{
  settling_ = false;
  settle_started_s_ = 0.0;
}

bool SegmentArrival::update(
  double now_s, double position_error, double heading_error,
  double linear_speed, double angular_speed, const Tolerances & tolerances)
{
  const double heading = std::abs(heading_error);
  const bool stopped = linear_speed <= tolerances.linear_speed &&
    angular_speed <= tolerances.angular_speed;
  const bool inside_loose = position_error <= tolerances.position_exit &&
    heading <= tolerances.heading_exit && stopped;
  const bool inside_tight = position_error <= tolerances.position &&
    heading <= tolerances.heading && stopped;

  if (!inside_loose) {
    reset();
    return false;
  }
  if (!inside_tight) {
    // Inside the loose band but not the tight one: hold, but do not finish.
    //
    // The settle already started is kept - that is what the two bands are for,
    // and restarting the clock every time the robot drifts across the tight
    // edge would mean a robot settling right on it never finishes. But the
    // segment is not completed from out here either: completing needs the
    // tight band, at the moment of completing, and the loose band only decides
    // how much drift is tolerated on the way there.
    return false;
  }
  if (!settling_) {
    settling_ = true;
    settle_started_s_ = now_s;
    return false;
  }
  return (now_s - settle_started_s_) >= tolerances.settle_s;
}

}  // namespace climbot_control
