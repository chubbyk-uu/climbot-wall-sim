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

#include "climbot_gazebo/total_station_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_gazebo
{

bool isLocalizationProfile(const std::string & profile)
{
  return profile == "precision" || profile == "realistic";
}

bool isComponentMode(const std::string & mode)
{
  return mode == "auto" || mode == "enabled" || mode == "disabled";
}

bool resolveComponentEnabled(const std::string & profile, const std::string & mode)
{
  if (!isLocalizationProfile(profile)) {
    throw std::invalid_argument(
            "localization_profile must be one of precision, realistic, not " + profile);
  }
  if (!isComponentMode(mode)) {
    throw std::invalid_argument(
            "component mode must be one of auto, enabled, disabled, not " + mode);
  }
  if (mode == "auto") {
    return profile == "realistic";
  }
  return mode == "enabled";
}

std::array<double, 3> rotateRobotResidualToWall(
  const std::array<double, 3> & residual_robot_m, double yaw_rad)
{
  const double cosine = std::cos(yaw_rad);
  const double sine = std::sin(yaw_rad);
  return {
    cosine * residual_robot_m[0] - sine * residual_robot_m[1],
    sine * residual_robot_m[0] + cosine * residual_robot_m[1],
    residual_robot_m[2]};
}

int64_t timestampWithClockErrorNs(int64_t source_ns, double correction_s)
{
  const auto correction_ns = static_cast<int64_t>(std::llround(correction_s * 1.0e9));
  return std::max<int64_t>(0, source_ns + correction_ns);
}

}  // namespace climbot_gazebo
