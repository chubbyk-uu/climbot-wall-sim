// Copyright 2026 jerry
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "climbot_gazebo/wall_surface.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <string>
namespace
{
std::string source()
{
  std::string text = "roughness:\n  modes:\n";
  for (int i = 0; i < 8; ++i) {
    text += "    - {period_m: 0.4, angle_rad: 0, phase_rad: 0, weight_m: 0}\n";
  }
  return text +
         R"(
grooves:
 - {start_m: [0, 0], end_m: [1, 0], width_m: 0.005, depth_range_m: [0.004, 0.004],
    variation_scale_m: 0.5, shoulder_fraction: 0.1, end_taper_m: 0, cross_section: trapezoid}
)";
}
}
TEST(WallSurface, IndependentTrapezoidCoordinatesAndNativeNormals)
{
  climbot_gazebo::WallSurface surface(source());
  EXPECT_DOUBLE_EQ(surface.height(.5, 0), -.004);
  EXPECT_NEAR(surface.height(.5, .00225), -.002, 1e-15);
  EXPECT_DOUBLE_EQ(surface.height(.5, .0025), 0);
  EXPECT_DOUBLE_EQ(surface.height(-.1, 0), 0);
  auto normal = surface.normal(.5, .00225);
  EXPECT_NEAR(normal[0], 0, 1e-10);
  EXPECT_NEAR(normal[1], -8 / std::sqrt(65.), 1e-10);
  EXPECT_NEAR(normal[2], 1 / std::sqrt(65.), 1e-10);
}
TEST(WallSurface, RuntimeRejectsInvalidResolvedGeometry)
{
  EXPECT_THROW(climbot_gazebo::WallSurface("roughness: {modes: []}\ngrooves: []"),
    std::invalid_argument);
}
