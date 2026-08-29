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

#ifndef CLIMBOT_DESCRIPTION__WALL_FRAME_HPP_
#define CLIMBOT_DESCRIPTION__WALL_FRAME_HPP_

#include <array>
#include <map>
#include <string>

#include "climbot_description/geometry.hpp"

namespace climbot_description
{

/// Right-handed wall work frame: +X along the wall, +Y up, +Z outward.
///
/// The origin is the wall's lower-left corner, so the working surface is
/// x in [0, width] and y in [0, height] and no wall coordinate is negative.
/// The stored pose is that of the wall frame expressed in the Gazebo world
/// frame, so positionFromWorld maps world coordinates into wall ones.
///
/// This is the single implementation of that transform. The Python package
/// binds to it rather than repeating it: every node that converts Gazebo truth
/// into wall coordinates has to agree, and two implementations of one
/// convention is how they stop agreeing.
class WallFrame
{
public:
  WallFrame(
    const std::array<double, 3> & origin_xyz,
    const std::array<double, 3> & origin_rpy,
    const std::map<std::string, double> & surface = {});

  /// Load a wall frame from a YAML file with a top-level `wall` key.
  static WallFrame fromYaml(const std::string & path);

  Vector3 positionFromWorld(const Vector3 & position) const;
  Quaternion orientationFromWorld(const Quaternion & quaternion) const;
  Quaternion rotationWorldFromWall() const {return world_from_wall_;}

  const std::array<double, 3> & origin() const {return origin_;}
  const std::array<double, 3> & rollPitchYaw() const {return roll_pitch_yaw_;}
  const std::map<std::string, double> & surface() const {return surface_;}

private:
  std::array<double, 3> origin_{};
  std::array<double, 3> roll_pitch_yaw_{};
  std::map<std::string, double> surface_;
  Quaternion world_from_wall_;
  Quaternion wall_from_world_;
};

}  // namespace climbot_description

#endif  // CLIMBOT_DESCRIPTION__WALL_FRAME_HPP_
