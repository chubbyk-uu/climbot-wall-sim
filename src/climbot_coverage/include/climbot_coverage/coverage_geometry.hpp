#ifndef CLIMBOT_COVERAGE__COVERAGE_GEOMETRY_HPP_
#define CLIMBOT_COVERAGE__COVERAGE_GEOMETRY_HPP_

#include <string>
#include <vector>

namespace climbot_coverage
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

using Polygon = std::vector<Point2>;

struct RegionResult
{
  Polygon polygon;
  double bottom_height_correction{0.0};
};

RegionResult makeRectangle(const Point2 & lower_left, const Point2 & upper_right);

RegionResult makeIsoscelesTrapezoid(
  const Point2 & lower_left, const Point2 & upper_right,
  const Point2 & lower_right);

Polygon insetConvexPolygon(const Polygon & polygon, double margin);

std::vector<Point2> generateBoustrophedonPath(
  const Polygon & polygon, double maximum_spacing,
  const std::string & sweep_direction, const std::string & start_corner);

double polygonArea(const Polygon & polygon);

bool approximatelyEqual(double first, double second, double tolerance = 1e-9);

}  // namespace climbot_coverage

#endif  // CLIMBOT_COVERAGE__COVERAGE_GEOMETRY_HPP_
