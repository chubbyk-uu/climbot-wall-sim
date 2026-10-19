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

TEST(SphereEnvelope, TwoOffsetsPreserveSingleAndCrossedGapSupport)
{
  using climbot_gazebo::SphereEnvelope;
  const std::vector<double> source{-0.04, -0.02, -0.0025, -0.00125, 0,
    0.00125, 0.0025, 0.02, 0.04};
  const std::vector<double> proxy{-0.02, -0.0025, -0.00125, 0, 0.00125, 0.0025, 0.02};
  for (bool cross : {false, true}) {
    std::vector<double> heights;
    for (double v : source) {
      for (double u : source) {
        heights.push_back(std::abs(v) < .0025 || (cross && std::abs(u) < .0025) ? -.004 : 0);
      }
    }
    SphereEnvelope original(source, source, heights);
    std::vector<double> intermediate;
    for (double v : proxy) {
      for (double u : proxy) {
        intermediate.push_back(original.height(u, v, .00625));
      }
    }
    SphereEnvelope reduced(proxy, proxy, intermediate);
    double expected = std::sqrt(.0125 * .0125 - .0025 * .0025 * (cross ? 2 : 1));
    EXPECT_NEAR(original.height(0, 0, .0125), expected, 1e-14);
    EXPECT_NEAR(reduced.height(0, 0, .00625), expected, 1e-14);
  }
}
TEST(SphereEnvelope, IncompleteFootprintsAndMalformedAxesFailExplicitly)
{
  using climbot_gazebo::SphereEnvelope;
  EXPECT_THROW(SphereEnvelope({0, 0}, {0, 1}, {0, 0, 0, 0}), std::invalid_argument);
  SphereEnvelope plane({-.02, 0, .02}, {-.02, 0, .02}, std::vector<double>(9, 0));
  EXPECT_DOUBLE_EQ(plane.height(0, 0, .01), .01);
  EXPECT_THROW(plane.height(.018, 0, .01), std::invalid_argument);
}
