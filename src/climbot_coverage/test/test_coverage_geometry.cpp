#include <algorithm>
#include <cmath>
#include <limits>
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
  const auto region = insetConvexPolygon(
    makeRectangle({-3.0, 0.5}, {3.0, 6.5}).polygon, 0.35);
  const auto path = generateBoustrophedonPath(
    region, 0.4, "horizontal", "lower_left");
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
  const auto region = insetConvexPolygon(
    makeIsoscelesTrapezoid({-3.0, 0.5}, {2.4, 6.5}, {3.0, 0.5}).polygon, 0.35);
  const auto path = generateBoustrophedonPath(
    region, 0.4, "vertical", "lower_left");
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
    generateBoustrophedonPath(rectangle, 0.0, "horizontal", "lower_left"),
    std::invalid_argument);
}

TEST(CoverageGeometry, CoversAtLeastNinetyEightPercentInBothDirections)
{
  constexpr double detection_width = 0.50;
  constexpr double overlap_ratio = 0.20;
  constexpr double row_spacing = detection_width * (1.0 - overlap_ratio);
  const std::vector<Polygon> regions{
    insetConvexPolygon(makeRectangle({-3.0, 0.5}, {3.0, 6.5}).polygon, 0.46),
    insetConvexPolygon(
      makeIsoscelesTrapezoid({-3.0, 0.5}, {2.4, 6.5}, {3.0, 0.5}).polygon, 0.46)};
  for (const auto & region : regions) {
    for (const auto & direction : {std::string("horizontal"), std::string("vertical")}) {
      const auto path = generateBoustrophedonPath(
        region, row_spacing, direction, "lower_left");
      EXPECT_GE(sampledCoverageRatio(region, path, detection_width), 0.98)
        << "direction=" << direction;
    }
  }
}

TEST(CoverageGeometry, IsExactlyDeterministicForIdenticalInput)
{
  const auto region = insetConvexPolygon(
    makeIsoscelesTrapezoid({-3.0, 0.5}, {2.4, 6.5}, {3.0, 0.5}).polygon, 0.46);
  const auto first = generateBoustrophedonPath(region, 0.4, "horizontal", "upper_right");
  const auto second = generateBoustrophedonPath(region, 0.4, "horizontal", "upper_right");
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_DOUBLE_EQ(first[index].x, second[index].x);
    EXPECT_DOUBLE_EQ(first[index].y, second[index].y);
  }
}

}  // namespace climbot_coverage
