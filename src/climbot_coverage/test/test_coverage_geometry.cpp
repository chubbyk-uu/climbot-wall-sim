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

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "climbot_coverage/coverage_geometry.hpp"
#include "gtest/gtest.h"

namespace climbot_coverage
{
namespace
{

bool insideConvex(const Polygon & polygon, const Point2 & point)
{
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto & first = polygon[index];
    const auto & second = polygon[(index + 1U) % polygon.size()];
    if ((second.x - first.x) * (point.y - first.y) -
      (second.y - first.y) * (point.x - first.x) < -1e-9)
    {
      return false;
    }
  }
  return true;
}

double distanceToSegment(const Point2 & point, const Point2 & first, const Point2 & second)
{
  const double delta_x = second.x - first.x;
  const double delta_y = second.y - first.y;
  const double length_squared = delta_x * delta_x + delta_y * delta_y;
  const double projection = std::clamp(
    ((point.x - first.x) * delta_x + (point.y - first.y) * delta_y) / length_squared,
    0.0, 1.0);
  return std::hypot(
    point.x - first.x - projection * delta_x,
    point.y - first.y - projection * delta_y);
}

double sampledCoverageRatio(
  const Polygon & polygon, const std::vector<Point2> & path, double detection_width)
{
  double minimum_x = std::numeric_limits<double>::max();
  double maximum_x = std::numeric_limits<double>::lowest();
  double minimum_y = std::numeric_limits<double>::max();
  double maximum_y = std::numeric_limits<double>::lowest();
  for (const auto & point : polygon) {
    minimum_x = std::min(minimum_x, point.x);
    maximum_x = std::max(maximum_x, point.x);
    minimum_y = std::min(minimum_y, point.y);
    maximum_y = std::max(maximum_y, point.y);
  }
  constexpr int samples_per_axis = 300;
  std::size_t inside = 0U;
  std::size_t covered = 0U;
  for (int row = 0; row < samples_per_axis; ++row) {
    for (int column = 0; column < samples_per_axis; ++column) {
      const Point2 sample{
        minimum_x + (maximum_x - minimum_x) * (column + 0.5) / samples_per_axis,
        minimum_y + (maximum_y - minimum_y) * (row + 0.5) / samples_per_axis};
      if (!insideConvex(polygon, sample)) {
        continue;
      }
      ++inside;
      for (std::size_t index = 0; index + 1U < path.size(); index += 2U) {
        if (distanceToSegment(sample, path[index], path[index + 1U]) <=
          0.5 * detection_width)
        {
          ++covered;
          break;
        }
      }
    }
  }
  return static_cast<double>(covered) / static_cast<double>(inside);
}

}  // namespace

TEST(CoverageGeometry, BuildsRectangleFromTwoPoints)
{
  const auto result = makeRectangle({-3.0, 0.5}, {3.0, 6.5});
  ASSERT_EQ(result.polygon.size(), 4U);
  EXPECT_DOUBLE_EQ(result.polygon[1].x, 3.0);
  EXPECT_DOUBLE_EQ(result.polygon[1].y, 0.5);
  EXPECT_DOUBLE_EQ(result.polygon[3].x, -3.0);
  EXPECT_DOUBLE_EQ(result.polygon[3].y, 6.5);
}

TEST(CoverageGeometry, CorrectsAndMirrorsIsoscelesTrapezoid)
{
  const auto result = makeIsoscelesTrapezoid(
    {-3.0, 0.46}, {2.4, 6.5}, {3.0, 0.54});
  ASSERT_EQ(result.polygon.size(), 4U);
  EXPECT_NEAR(result.polygon[0].y, 0.5, 1e-12);
  EXPECT_NEAR(result.polygon[1].y, 0.5, 1e-12);
  EXPECT_NEAR(result.polygon[3].x, -2.4, 1e-12);
  EXPECT_NEAR(result.bottom_height_correction, 0.08, 1e-12);
}

TEST(CoverageGeometry, InsetsTrapezoidAlongEveryEdge)
{
  const auto region = makeIsoscelesTrapezoid(
    {-3.0, 0.5}, {2.4, 6.5}, {3.0, 0.5});
  const auto inset = insetConvexPolygon(region.polygon, 0.35);
  ASSERT_EQ(inset.size(), 4U);
  EXPECT_GT(polygonArea(inset), 0.0);
  EXPECT_GT(inset[0].x, region.polygon[0].x);
  EXPECT_LT(inset[1].x, region.polygon[1].x);
  EXPECT_GT(inset[0].y, region.polygon[0].y);
  EXPECT_LT(inset[2].y, region.polygon[2].y);
}

TEST(CoverageGeometry, GeneratesHorizontalStraightAlternatingLines)
{
  const auto coverage = makeRectangle({-3.0, 0.5}, {3.0, 6.5}).polygon;
  const auto motion = makeRectangle({-3.1, 0.25}, {3.1, 6.75}).polygon;
  const auto path = generateFootprintAwareBoustrophedonPath(
    coverage, motion, 0.5, 0.4, "horizontal", "lower_left");
  ASSERT_GE(path.size(), 4U);
  for (std::size_t index = 0; index < path.size(); index += 2) {
    EXPECT_NEAR(path[index].y, path[index + 1].y, 1e-9);
    if ((index / 2U) % 2U == 0U) {
      EXPECT_LT(path[index].x, path[index + 1].x);
    } else {
      EXPECT_GT(path[index].x, path[index + 1].x);
    }
  }
}

TEST(CoverageGeometry, GeneratesVerticalStraightAlternatingLines)
{
  const auto coverage = makeIsoscelesTrapezoid(
    {-3.0, 0.5}, {2.4, 6.5}, {3.0, 0.5}).polygon;
  const auto motion = makeRectangle({-3.5, 0.0}, {3.5, 7.0}).polygon;
  const auto path = generateFootprintAwareBoustrophedonPath(
    coverage, motion, 0.5, 0.4, "vertical", "lower_left");
  ASSERT_GE(path.size(), 4U);
  for (std::size_t index = 0; index < path.size(); index += 2) {
    EXPECT_NEAR(path[index].x, path[index + 1].x, 1e-9);
    if ((index / 2U) % 2U == 0U) {
      EXPECT_LT(path[index].y, path[index + 1].y);
    } else {
      EXPECT_GT(path[index].y, path[index + 1].y);
    }
  }
}

TEST(CoverageGeometry, RejectsInvalidRegionAndSpacing)
{
  EXPECT_THROW(makeRectangle({1.0, 1.0}, {0.0, 2.0}), std::invalid_argument);
  const auto rectangle = makeRectangle({0.0, 0.0}, {2.0, 2.0}).polygon;
  EXPECT_THROW(
    generateFootprintAwareBoustrophedonPath(
      rectangle, rectangle, 0.5, 0.0, "horizontal", "lower_left"),
    std::invalid_argument);
  EXPECT_THROW(
    generateFootprintAwareBoustrophedonPath(
      {}, rectangle, 0.5, 0.4, "horizontal", "lower_left"),
    std::invalid_argument);
}

TEST(CoverageGeometry, CoversAtLeastNinetyEightPercentInBothDirections)
{
  constexpr double detection_width = 0.50;
  constexpr double overlap_ratio = 0.20;
  constexpr double row_spacing = detection_width * (1.0 - overlap_ratio);
  const std::vector<Polygon> regions{
    makeRectangle({-3.0, 0.5}, {3.0, 6.5}).polygon,
    makeIsoscelesTrapezoid({-3.0, 0.5}, {2.4, 6.5}, {3.0, 0.5}).polygon};
  const auto motion = makeRectangle({-4.0, 0.0}, {4.0, 7.0}).polygon;
  for (const auto & region : regions) {
    for (const auto & direction : {std::string("horizontal"), std::string("vertical")}) {
      const auto path = generateFootprintAwareBoustrophedonPath(
        region, motion, detection_width, row_spacing, direction, "lower_left");
      EXPECT_GE(sampledCoverageRatio(region, path, detection_width, 0.1), 0.98)
        << "direction=" << direction;
    }
  }
}

TEST(CoverageGeometry, FootprintAwarePathCoversRequestedRegionInsideMotionRegion)
{
  constexpr double detection_width = 0.50;
  constexpr double detection_length = 0.10;
  const auto coverage = makeRectangle({-3.0, 0.75}, {3.0, 6.5}).polygon;
  const auto motion = makeRectangle({-4.0, 0.55}, {4.0, 7.2}).polygon;
  for (const auto & direction : {std::string("horizontal"), std::string("vertical")}) {
    const auto path = generateFootprintAwareBoustrophedonPath(
      coverage, motion, detection_width, 0.4, direction, "lower_left");
    EXPECT_GE(sampledCoverageRatio(coverage, path, detection_width, detection_length), 0.98)
      << "direction=" << direction;
    for (const auto & waypoint : path) {
      EXPECT_TRUE(insideConvex(motion, waypoint));
    }
  }
}

TEST(CoverageGeometry, CameraProjectionDoesNotMoveTheRobotPath)
{
  constexpr double camera_offset = 0.30;
  const auto coverage = makeRectangle({1.0, 1.0}, {5.0, 3.0}).polygon;
  const auto motion = makeRectangle({0.0, 0.0}, {6.0, 4.0}).polygon;
  const auto camera = generateFootprintAwareBoustrophedonPath(
    coverage, motion, 0.50, 0.40, "horizontal", "lower_left");
  EXPECT_GE(sampledCoverageRatio(
      coverage, camera, 0.50, 0.28125, 300, camera_offset), 0.965);
}

TEST(CoverageGeometry, RobotEndpointsStayOnTheSelectedDriveBoundary)
{
  const auto coverage = makeRectangle({1.0, 1.0}, {5.0, 2.0}).polygon;
  const auto motion = makeRectangle({0.0, 0.0}, {6.0, 3.0}).polygon;
  const auto path = generateFootprintAwareBoustrophedonPath(
    coverage, motion, 0.50, 0.40, "horizontal", "lower_left");
  ASSERT_GE(path.size(), 4U);
  EXPECT_NEAR(path.front().x, 1.0, 1e-12);
  EXPECT_NEAR(path.front().y, 1.0 + 0.25, 1e-12);
  EXPECT_NEAR(path[1].x, 5.0, 1e-12);
  EXPECT_NEAR(path[path.size() - 2U].y, 2.0 - 0.25, 1e-12);
}

TEST(CoverageGeometry, CameraGeometryNeverExpandsTheRobotDrivePath)
{
  constexpr double camera_offset = 0.340;
  constexpr double detection_length = 0.28125;
  const auto motion = makeRectangle({0.0, 0.0}, {10.0, 4.0}).polygon;
  const auto drive = makeRectangle({0.2, 0.5}, {9.8, 3.5}).polygon;
  const auto path = generateFootprintAwareBoustrophedonPath(
    drive, motion, 0.50, 0.40, "horizontal", "lower_left");
  for (const auto & waypoint : path) {
    EXPECT_TRUE(insideConvex(drive, waypoint));
    EXPECT_TRUE(insideConvex(motion, waypoint));
  }
  EXPECT_GE(sampledCoverageRatio(
      drive, path, 0.50, detection_length, 300, camera_offset), 0.965);
}

TEST(CoverageGeometry, KeepsRouteInsideDriveRegionForBothSweeps)
{
  const auto drive = makeIsoscelesTrapezoid({0.4, 0.8}, {7.8, 4.1}, {8.8, 0.8}).polygon;
  const auto motion = makeRectangle({0.0, 0.5}, {9.2, 4.5}).polygon;
  for (const auto & direction : {std::string("horizontal"), std::string("vertical")}) {
    const auto path = generateFootprintAwareBoustrophedonPath(
      drive, motion, 0.50, 0.40, direction, "lower_left");
    ASSERT_FALSE(path.empty());
    for (const auto & waypoint : path) {
      EXPECT_TRUE(insideConvex(drive, waypoint));
    }
  }
}

TEST(CoverageGeometry, KeepsExactMultipleOfMaximumSpacingAtTheExpectedLineCount)
{
  constexpr double detection_width = 0.50;
  constexpr double maximum_spacing = 0.40;
  const double height = std::nextafter(1.70, std::numeric_limits<double>::infinity());
  const auto coverage = makeRectangle({0.0, 0.0}, {4.0, height}).polygon;
  const auto motion = makeRectangle({-0.10, -0.10}, {4.10, height + 0.10}).polygon;
  const auto path = generateFootprintAwareBoustrophedonPath(
    coverage, motion, detection_width, maximum_spacing,
    "horizontal", "lower_left");
  ASSERT_EQ(path.size(), 8U);
  for (std::size_t index = 0; index < path.size(); index += 2U) {
    EXPECT_NEAR(path[index].y, 0.25 + 0.40 * static_cast<double>(index / 2U), 1e-12);
    EXPECT_NEAR(path[index].y, path[index + 1U].y, 1e-12);
  }
}

TEST(CoverageGeometry, RejectsFootprintThatCannotRemainInsideMotionRegion)
{
  const auto coverage = makeRectangle({0.0, 0.0}, {2.0, 2.0}).polygon;
  const auto motion = makeRectangle({0.1, 0.1}, {1.9, 1.9}).polygon;
  EXPECT_THROW(
    generateFootprintAwareBoustrophedonPath(
      coverage, motion, 0.5, 0.4, "horizontal", "lower_left"),
    std::invalid_argument);
}

TEST(CoverageGeometry, RejectsInvalidFootprintPathOptions)
{
  const auto region = makeRectangle({0.0, 0.0}, {2.0, 2.0}).polygon;
  EXPECT_THROW(
    generateFootprintAwareBoustrophedonPath(
      region, region, 0.5, 0.4, "horizontal", "banana"),
    std::invalid_argument);
  EXPECT_THROW(
    generateFootprintAwareBoustrophedonPath(
      region, region, 0.5, 0.4, "horizontal", "ab"),
    std::invalid_argument);
  EXPECT_THROW(
    generateFootprintAwareBoustrophedonPath(
      region, region, 0.5, 0.0, "horizontal", "lower_left"),
    std::invalid_argument);
}

TEST(CoverageGeometry, IsExactlyDeterministicForIdenticalInput)
{
  const auto coverage = makeIsoscelesTrapezoid(
    {-3.0, 0.5}, {2.4, 6.5}, {3.0, 0.5}).polygon;
  const auto motion = makeRectangle({-4.0, 0.0}, {4.0, 7.0}).polygon;
  const auto first = generateFootprintAwareBoustrophedonPath(
    coverage, motion, 0.5, 0.4, "horizontal", "upper_right");
  const auto second = generateFootprintAwareBoustrophedonPath(
    coverage, motion, 0.5, 0.4, "horizontal", "upper_right");
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_DOUBLE_EQ(first[index].x, second[index].x);
    EXPECT_DOUBLE_EQ(first[index].y, second[index].y);
  }
}

TEST(TopEdgeFinishingScan, SweepsTheStripAVerticalPassLeavesAtTheTop)
{
  const auto coverage = makeRectangle({0.0, 2.0}, {3.0, 6.5}).polygon;
  const auto motion = makeRectangle({-1.0, 1.0}, {4.0, 7.5}).polygon;
  const auto line = makeTopEdgeFinishingScan(coverage, motion, {3.0, 6.5});
  ASSERT_EQ(line.size(), 2U);
  EXPECT_NEAR(line.front().y, 6.5, 1e-9);
  EXPECT_NEAR(line.back().y, 6.5, 1e-9);
  EXPECT_NEAR(std::abs(line.back().x - line.front().x), 3.0, 1e-9);
}

TEST(TopEdgeFinishingScan, ClosesTheGapAVerticalSweepLeaves)
{
  const auto coverage = makeRectangle({0.0, 2.0}, {3.0, 6.5}).polygon;
  const auto motion = makeRectangle({-1.0, 1.0}, {4.0, 7.5}).polygon;
  // A deliberately short footprint along travel leaves a strip at the column
  // ends, which is exactly the case 10.7 asks the finishing scan to close.
  auto path = generateFootprintAwareBoustrophedonPath(
    coverage, motion, 0.5, 0.4, "vertical", "lower_left");
  const double before = sampledCoverageRatio(coverage, path, 0.5, 0.01);
  const auto line = makeTopEdgeFinishingScan(coverage, motion, path.back());
  ASSERT_EQ(line.size(), 2U);
  path.insert(path.end(), line.begin(), line.end());
  EXPECT_GE(sampledCoverageRatio(coverage, path, 0.5, 0.01), before);
}

TEST(TopEdgeFinishingScan, IsEnteredFromTheEndNearerTheLastScan)
{
  const auto coverage = makeRectangle({0.0, 2.0}, {3.0, 6.5}).polygon;
  const auto motion = makeRectangle({-1.0, 1.0}, {4.0, 7.5}).polygon;
  const auto from_right = makeTopEdgeFinishingScan(
    coverage, motion, {3.0, 6.5});
  const auto from_left = makeTopEdgeFinishingScan(
    coverage, motion, {0.0, 6.5});
  ASSERT_EQ(from_right.size(), 2U);
  ASSERT_EQ(from_left.size(), 2U);
  EXPECT_GT(from_right.front().x, from_right.back().x);
  EXPECT_LT(from_left.front().x, from_left.back().x);
}

TEST(TopEdgeFinishingScan, RefusesALineThatLeavesMotionRegion)
{
  const auto coverage = makeRectangle({0.0, 2.0}, {3.0, 6.5}).polygon;
  // Too narrow for the line ends, which must sit inside it (10.7).
  const auto motion = makeRectangle({1.0, 1.0}, {2.0, 7.5}).polygon;
  EXPECT_TRUE(makeTopEdgeFinishingScan(coverage, motion, {3.0, 6.5}).empty());
}

TEST(TopEdgeFinishingScan, CentresOnARegionShorterThanTheFootprint)
{
  const auto coverage = makeRectangle({0.0, 2.0}, {3.0, 2.3}).polygon;
  const auto motion = makeRectangle({-1.0, 1.0}, {4.0, 7.5}).polygon;
  const auto line = makeTopEdgeFinishingScan(coverage, motion, {3.0, 2.3});
  ASSERT_EQ(line.size(), 2U);
  EXPECT_NEAR(line.front().y, 2.3, 1e-9);
}

TEST(TopEdgeFinishingScan, RejectsInvalidRegions)
{
  const auto region = makeRectangle({0.0, 0.0}, {3.0, 4.0}).polygon;
  EXPECT_THROW(
    makeTopEdgeFinishingScan({}, region, {0.0, 0.0}),
    std::invalid_argument);
}

}  // namespace climbot_coverage
