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

#ifndef CLIMBOT_CONTROL__SEGMENT_ARRIVAL_HPP_
#define CLIMBOT_CONTROL__SEGMENT_ARRIVAL_HPP_

namespace climbot_control
{

/// Decides when a segment has been arrived at, and does not change its mind.
///
/// Three things have to hold together, and each is here for a reason:
///
/// A tight band to enter on and a loose one to leave on. One band alone makes
/// the decision flicker for a robot sitting on the edge of it, and every flick
/// is a segment reported finished and then unfinished.
///
/// A settle time, because the robot slips while stopped on a wall. Arriving is
/// not the same as staying, and a ball the robot leaves as fast as it enters
/// is not arrival.
///
/// Both speeds under their limits, because a robot passing through the goal at
/// speed satisfies every position test on its way past.
class SegmentArrival
{
public:
  struct Tolerances
  {
    /// Entering: all three must hold to start the settle timer.
    double position{0.0};
    double heading{0.0};
    /// Leaving: exceeding any of these abandons the settle and starts over.
    /// Both must be looser than their counterparts above.
    double position_exit{0.0};
    double heading_exit{0.0};
    /// Above either of these the robot counts as still moving.
    double linear_speed{0.0};
    double angular_speed{0.0};
    /// How long the tight band has to hold before the segment is complete.
    double settle_s{0.0};
  };

  /// Feed one control cycle. True when the segment is complete.
  ///
  /// now_s is any monotonic clock in seconds; only differences are used.
  bool update(
    double now_s, double position_error, double heading_error,
    double linear_speed, double angular_speed, const Tolerances & tolerances);

  /// Whether the settle timer is running: inside the tight band, not yet held
  /// for long enough.
  bool settling() const {return settling_;}

  /// Forget any settle in progress, for a new segment or an abandoned one.
  void reset();

private:
  bool settling_{false};
  double settle_started_s_{0.0};
};

}  // namespace climbot_control

#endif  // CLIMBOT_CONTROL__SEGMENT_ARRIVAL_HPP_
