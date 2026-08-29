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

#ifndef CLIMBOT_GAZEBO__TOTAL_STATION_MODEL_HPP_
#define CLIMBOT_GAZEBO__TOTAL_STATION_MODEL_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace climbot_gazebo
{

/// Deterministic pieces of the total-station measurement model.
///
/// Free of ROS messages so the physical convention and timestamp arithmetic
/// can be checked directly. The node owns parameter plumbing and publication
/// order.

bool isLocalizationProfile(const std::string & profile);
bool isComponentMode(const std::string & mode);

/// Resolve one independently-overridable component of a named profile.
bool resolveComponentEnabled(const std::string & profile, const std::string & mode);

/// Rotate a robot-frame prism residual into wall work coordinates.
///
/// The robot and wall frames share +Z as the wall normal. The residual's first
/// two components are therefore rotated by truth yaw in the wall plane; this
/// makes the position error reverse direction when the robot reverses.
std::array<double, 3> rotateRobotResidualToWall(
  const std::array<double, 3> & residual_robot_m, double yaw_rad);

/// Return a header timestamp carrying clock bias and independent jitter only.
///
/// builtin_interfaces/Time cannot represent a negative nanosecond value, so a
/// negative result is clamped. That only affects the first moment of a
/// negative-bias simulation; delivery scheduling continues to use the source
/// time, never this stamped value.
int64_t timestampWithClockErrorNs(int64_t source_ns, double correction_s);

}  // namespace climbot_gazebo

#endif  // CLIMBOT_GAZEBO__TOTAL_STATION_MODEL_HPP_
