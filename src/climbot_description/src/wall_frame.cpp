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

#include "climbot_description/wall_frame.hpp"

#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace climbot_description
{

namespace
{

std::array<double, 3> triple(
  const YAML::Node & node, const std::string & path,
  const std::string & key)
{
  if (!node || !node.IsSequence() || node.size() != 3U) {
    throw std::invalid_argument(path + " wall." + key + " needs three values.");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

}  // namespace

WallFrame::WallFrame(
  const std::array<double, 3> & origin_xyz,
  const std::array<double, 3> & origin_rpy,
  const std::map<std::string, double> & surface)
: origin_(origin_xyz),
  roll_pitch_yaw_(origin_rpy),
  surface_(surface),
  world_from_wall_(quaternionFromRpy(origin_rpy[0], origin_rpy[1], origin_rpy[2])),
  wall_from_world_(quaternionConjugate(world_from_wall_))
{
}

WallFrame WallFrame::fromYaml(const std::string & path)
{
  YAML::Node document;
  try {
    document = YAML::LoadFile(path);
  } catch (const YAML::Exception & exception) {
    throw std::invalid_argument(path + " could not be read: " + exception.what());
  }
  if (!document.IsMap() || !document["wall"]) {
    throw std::invalid_argument(path + " has no top-level \"wall\" section.");
  }
  const YAML::Node wall = document["wall"];
  for (const char * key : {"origin_xyz", "origin_rpy"}) {
    if (!wall[key]) {
      throw std::invalid_argument(path + " is missing wall." + key + ".");
    }
  }
  std::map<std::string, double> surface;
  if (wall["surface"] && wall["surface"].IsMap()) {
    for (const auto & entry : wall["surface"]) {
      try {
        surface[entry.first.as<std::string>()] = entry.second.as<double>();
      } catch (const YAML::Exception &) {
        // The Python dictionary carried non-numeric surface entries too, but
        // no caller of this transform reads them. Skipping keeps a descriptive
        // string in the description from failing every node that loads it.
        continue;
      }
    }
  }
  return WallFrame(
    triple(wall["origin_xyz"], path, "origin_xyz"),
    triple(wall["origin_rpy"], path, "origin_rpy"), surface);
}

Vector3 WallFrame::positionFromWorld(const Vector3 & position) const
{
  const Vector3 offset{
    position.x - origin_[0], position.y - origin_[1], position.z - origin_[2]};
  return rotateVector(wall_from_world_, offset);
}

Quaternion WallFrame::orientationFromWorld(const Quaternion & quaternion) const
{
  return quaternionMultiply(wall_from_world_, quaternion);
}

}  // namespace climbot_description
