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

// Generates SCAN line pairs whose rectangular swept detection footprint covers
// coverage_region.  Every robot-centre waypoint must lie in motion_region.
std::vector<Point2> generateFootprintAwareBoustrophedonPath(
  const Polygon & coverage_region, const Polygon & motion_region,
  double detection_width, double detection_length, double maximum_spacing,
  const std::string & sweep_direction, const std::string & start_corner);

// Estimates the fraction of coverage_region swept by SCAN line pairs.  Each
// pair is interpreted as a straight segment with a rectangular footprint.
double sampledCoverageRatio(
  const Polygon & coverage_region, const std::vector<Point2> & scan_path,
  double detection_width, double detection_length, int samples_per_axis = 300);

double polygonArea(const Polygon & polygon);

bool approximatelyEqual(double first, double second, double tolerance = 1e-9);

}  // namespace climbot_coverage

#endif  // CLIMBOT_COVERAGE__COVERAGE_GEOMETRY_HPP_
