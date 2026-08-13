#include <cmath>
#include <string>
#include <vector>

#include "climbot_coverage/coverage_geometry.hpp"
#include "gtest/gtest.h"

namespace climbot_coverage
{

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

}  // namespace climbot_coverage
