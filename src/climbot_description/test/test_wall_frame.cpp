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

#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

#include "climbot_description/wall_frame.hpp"

using climbot_description::Quaternion;
using climbot_description::Vector3;
using climbot_description::WallFrame;

namespace
{

std::string writeTemporary(const std::string & contents)
{
  // mkstemp rather than tmpnam: the name tmpnam returns can be claimed by
  // another process before this one opens it.
  char pattern[] = "/tmp/climbot_wall_frame_XXXXXX";
  const int descriptor = mkstemp(pattern);
  if (descriptor < 0) {
    throw std::runtime_error("could not create a temporary file");
  }
  ::close(descriptor);
  const std::string path(pattern);
  std::ofstream handle(path);
  handle << contents;
  handle.close();
  return path;
}

constexpr double kHalfPi = 1.57079632679489661923;

}  // namespace

TEST(WallFrameCpp, MapsTheOriginToZero)
{
  const WallFrame frame({1.0, -2.0, 0.5}, {0.0, 0.0, 0.0});
  const Vector3 wall = frame.positionFromWorld(Vector3{1.0, -2.0, 0.5});
  EXPECT_NEAR(wall.x, 0.0, 1e-12);
  EXPECT_NEAR(wall.y, 0.0, 1e-12);
  EXPECT_NEAR(wall.z, 0.0, 1e-12);
}

TEST(WallFrameCpp, UndoesTheWallRotation)
{
  // A wall yawed a quarter turn: world +Y is the wall's +X.
  const WallFrame frame({0.0, 0.0, 0.0}, {0.0, 0.0, kHalfPi});
  const Vector3 wall = frame.positionFromWorld(Vector3{0.0, 2.0, 0.0});
  EXPECT_NEAR(wall.x, 2.0, 1e-12);
  EXPECT_NEAR(wall.y, 0.0, 1e-12);
  EXPECT_NEAR(wall.z, 0.0, 1e-12);
}

TEST(WallFrameCpp, OrientationFromWorldCancelsTheFrameRotation)
{
  const WallFrame frame({0.0, 0.0, 0.0}, {0.0, 0.0, kHalfPi});
  const Quaternion wall = frame.orientationFromWorld(frame.rotationWorldFromWall());
  EXPECT_NEAR(wall.x, 0.0, 1e-12);
  EXPECT_NEAR(wall.y, 0.0, 1e-12);
  EXPECT_NEAR(wall.z, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(wall.w), 1.0, 1e-12);
}

TEST(WallFrameCpp, LoadsOriginAndSurfaceFromYaml)
{
  const std::string path = writeTemporary(
    "wall:\n"
    "  origin_xyz: [1.0, 2.0, 3.0]\n"
    "  origin_rpy: [0.0, 0.0, 0.0]\n"
    "  surface:\n"
    "    width_m: 10.0\n"
    "    height_m: 8.0\n");
  const WallFrame frame = WallFrame::fromYaml(path);
  EXPECT_NEAR(frame.origin()[0], 1.0, 1e-12);
  EXPECT_NEAR(frame.origin()[2], 3.0, 1e-12);
  EXPECT_NEAR(frame.surface().at("width_m"), 10.0, 1e-12);
  EXPECT_NEAR(frame.surface().at("height_m"), 8.0, 1e-12);
  std::remove(path.c_str());
}

TEST(WallFrameCpp, RejectsADocumentWithoutAWallSection)
{
  const std::string path = writeTemporary("robot:\n  name: climbot\n");
  EXPECT_THROW(WallFrame::fromYaml(path), std::invalid_argument);
  std::remove(path.c_str());
}

TEST(WallFrameCpp, RejectsAWallMissingItsOrigin)
{
  const std::string path = writeTemporary("wall:\n  origin_xyz: [0.0, 0.0, 0.0]\n");
  EXPECT_THROW(WallFrame::fromYaml(path), std::invalid_argument);
  std::remove(path.c_str());
}

TEST(WallFrameCpp, RejectsAnOriginThatIsNotThreeValues)
{
  const std::string path = writeTemporary(
    "wall:\n  origin_xyz: [0.0, 0.0]\n  origin_rpy: [0.0, 0.0, 0.0]\n");
  EXPECT_THROW(WallFrame::fromYaml(path), std::invalid_argument);
  std::remove(path.c_str());
}

TEST(WallFrameCpp, RejectsAMissingFile)
{
  EXPECT_THROW(WallFrame::fromYaml("/nonexistent/wall.yaml"), std::invalid_argument);
}
