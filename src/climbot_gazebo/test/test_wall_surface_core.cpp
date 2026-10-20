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

namespace
{
std::string channelSource()
{
  auto text = source();
  auto begin = text.find("start_m: [0, 0], end_m: [1, 0]");
  text.replace(begin, std::string("start_m: [0, 0], end_m: [1, 0]").size(),
    "start_m: [0, 1], end_m: [1, 1]");return text;
}
using Corners = std::array<std::array<double, 3>, 4>;
Corners ring(double angle = 0)
{
  Corners corners{{{.3, .9, .018}, {.7, .9, .018}, {.7, 1.1, .018}, {.3, 1.1, .018}}};
  for (auto & p : corners) {
    double x = p[0] - .5, y = p[1] - 1;
    p[0] = .5 + x * std::cos(angle) - y * std::sin(angle);
    p[1] = 1 + x * std::sin(angle) + y * std::cos(angle);
  }
  return corners;
}
}
TEST(SealChannels, TrapezoidApertureMatchesIndependentIntegralAndRefinement)
{
  climbot_gazebo::WallSurface s(channelSource());
  // Two crossings, each with 4 mm flat bottom plus two 0.5 mm shoulders.
  double x = .004 - .001, width = .0001;
  double primitive = x * x / 2 - width * x / 2 + width * width / 6;
  double expected = 2 * (.004 * (x - width / 2) + 2 * .0005 / .004 * primitive);
  auto coarse = s.sealLeak(ring(), .02, .022, .001, width, {2, 2});
  auto fine = s.sealLeak(ring(), .02, .022, .001, width, {2, 2}, 4);
  EXPECT_NEAR(coarse.area_m2, expected, expected * .01);
  EXPECT_NEAR(coarse.area_m2, fine.area_m2, expected * .01);
  EXPECT_DOUBLE_EQ(coarse.area_m2, coarse.channel_area_m2);
  EXPECT_GT(coarse.evaluations, 0u);
}
TEST(SealChannels, FinalEndpointCannotRoundOutsideAndOmitAnEdgeTail)
{
  auto text = source();text = text.substr(0, text.find("grooves:")) + "grooves: []\n";
  climbot_gazebo::WallSurface s(text);
  Corners c{{{.8434144045362073, .38079047857754444, .024},
    {1.1190031995429564, .6796075089745737, .024},
    {.8823001722544711, .8979088374992943, .024},
    {.606711377247722, .5990918071022653, .024}}};
  double perimeter = 0;
  for (int i = 0; i < 4; ++i) {
    perimeter += std::hypot(c[i][0] - c[(i + 1) % 4][0], c[i][1] - c[(i + 1) % 4][1]);
  }
  double expected = perimeter * (.024 - .022 - .0001 / 2);
  for (int refinement : {1, 2, 4, 8}) {
    EXPECT_NEAR(s.sealLeak(c, .02, .022, 0, .0001, {1.6, 1.6}, refinement).area_m2,
      expected, 1e-14);
  }
}
TEST(SealChannels, ObliqueCrossingAndDuplicateSourcesDoNotInflateFlowArea)
{
  auto text = channelSource();climbot_gazebo::WallSurface s(text);
  auto duplicateText = text + text.substr(text.find(" - {", text.find("grooves:")));
  climbot_gazebo::WallSurface duplicate(duplicateText);
  double normal = s.sealLeak(ring(), .02, .022, .001, .0001, {2, 2}).area_m2;
  EXPECT_NEAR(s.sealLeak(ring(.4), .02, .022, .001, .0001, {2, 2}).area_m2,
    normal, normal * .01);
  EXPECT_NEAR(duplicate.sealLeak(ring(), .02, .022, .001, .0001, {2, 2}).area_m2,
    normal, 1e-14);
}
TEST(SealChannels, FinitePitDoesNotAutomaticallyConnectThroughTheBand)
{
  auto text = channelSource();auto begin = text.find("start_m: [0, 1], end_m: [1, 1]");
  text.replace(begin, std::string("start_m: [0, 1], end_m: [1, 1]").size(),
    "start_m: [0.295, 1], end_m: [0.305, 1]");
  climbot_gazebo::WallSurface s(text);
  EXPECT_DOUBLE_EQ(s.sealLeak(ring(), .02, .022, .001, .0001, {2, 2}).area_m2, 0);
}
TEST(SealChannels, NoIntrusionIsConservativeAndLiftTransitionHasNoJump)
{
  climbot_gazebo::WallSurface s(channelSource());
  double previous = 0;
  for (int i = 0; i <= 800; ++i) {
    auto c = ring();for (auto & p : c) {
      p[2] = .018 + .00001 * i;
    }
    double area = s.sealLeak(c, .02, .022, .001, .0001, {2, 2}).area_m2;
    double conservative = s.sealLeak(c, .02, .022, 0, .0001, {2, 2}).area_m2;
    EXPECT_LE(area, conservative + 1e-14);
    if (i) {EXPECT_LE(std::abs(area - previous), .000013);}
    previous = area;
  }
}
TEST(SealChannels, NarrowFeatureIsNotAliasedByCoarsePerimeterPhases)
{
  climbot_gazebo::WallSurface s(channelSource());
  double expected = s.sealLeak(ring(), .02, .022, .001, .0001, {2, 2}).area_m2;
  for (int i = 1; i <= 20; ++i) {
    auto c = ring();for (auto & p : c) {
      p[1] += .0004 * i;
    }
    EXPECT_NEAR(s.sealLeak(c, .02, .022, .001, .0001, {2, 2}).area_m2,
      expected, expected * .01);
  }
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
