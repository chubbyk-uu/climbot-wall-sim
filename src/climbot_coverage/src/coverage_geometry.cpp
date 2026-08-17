#include "climbot_coverage/coverage_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace climbot_coverage
{
namespace
{

void validateStartCorner(const std::string & start_corner)
{
  if (start_corner != "lower_left" && start_corner != "lower_right" &&
    start_corner != "upper_left" && start_corner != "upper_right")
  {
    throw std::invalid_argument("Unsupported start corner.");
  }
}

constexpr double kEpsilon = 1e-9;
constexpr double kLineCountTolerance = 1e-9;

Point2 subtract(const Point2 & first, const Point2 & second)
{
  return {first.x - second.x, first.y - second.y};
}

Point2 add(const Point2 & first, const Point2 & second)
{
  return {first.x + second.x, first.y + second.y};
}

Point2 scale(const Point2 & point, double scalar)
{
  return {point.x * scalar, point.y * scalar};
}

double cross(const Point2 & first, const Point2 & second)
{
  return first.x * second.y - first.y * second.x;
}

Point2 intersectInfiniteLines(
  const Point2 & first_origin, const Point2 & first_direction,
  const Point2 & second_origin, const Point2 & second_direction)
{
  const double denominator = cross(first_direction, second_direction);
  if (std::abs(denominator) < kEpsilon) {
    throw std::invalid_argument("Adjacent offset edges are parallel.");
  }
  const double parameter = cross(
    subtract(second_origin, first_origin), second_direction) / denominator;
  return add(first_origin, scale(first_direction, parameter));
}

std::pair<double, double> bounds(
  const Polygon & polygon, bool horizontal_sweep)
{
  double minimum = horizontal_sweep ? polygon.front().y : polygon.front().x;
  double maximum = minimum;
  for (const auto & point : polygon) {
    const double coordinate = horizontal_sweep ? point.y : point.x;
    minimum = std::min(minimum, coordinate);
    maximum = std::max(maximum, coordinate);
  }
  return {minimum, maximum};
}

std::pair<Point2, Point2> clipScanLine(
  const Polygon & polygon, double coordinate, bool horizontal_sweep)
{
  std::vector<double> intersections;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const Point2 & first = polygon[index];
    const Point2 & second = polygon[(index + 1) % polygon.size()];
    const double first_cross = horizontal_sweep ? first.y : first.x;
    const double second_cross = horizontal_sweep ? second.y : second.x;
    if (coordinate < std::min(first_cross, second_cross) - kEpsilon ||
      coordinate > std::max(first_cross, second_cross) + kEpsilon)
    {
      continue;
    }
    const double difference = second_cross - first_cross;
    if (std::abs(difference) < kEpsilon) {
      continue;
    }
    const double ratio = (coordinate - first_cross) / difference;
    const double along = horizontal_sweep ?
      first.x + ratio * (second.x - first.x) :
      first.y + ratio * (second.y - first.y);
    intersections.push_back(along);
  }
  std::sort(intersections.begin(), intersections.end());
  intersections.erase(
    std::unique(
      intersections.begin(), intersections.end(),
      [](double first, double second) {return approximatelyEqual(first, second, 1e-7);}),
    intersections.end());
  if (intersections.size() < 2) {
    throw std::invalid_argument("A scan line does not cross the effective region twice.");
  }
  if (horizontal_sweep) {
    return {{intersections.front(), coordinate}, {intersections.back(), coordinate}};
  }
  return {{coordinate, intersections.front()}, {coordinate, intersections.back()}};
}

bool insideConvex(const Polygon & polygon, const Point2 & point)
{
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const Point2 & first = polygon[index];
    const Point2 & second = polygon[(index + 1) % polygon.size()];
    if (cross(subtract(second, first), subtract(point, first)) < -kEpsilon) {
      return false;
    }
  }
  return true;
}

bool coveredByFootprint(
  const Point2 & sample, const Point2 & first, const Point2 & second,
  double detection_width, double detection_length)
{
  const Point2 direction = subtract(second, first);
  const double segment_length = std::hypot(direction.x, direction.y);
  if (segment_length < kEpsilon) {
    return false;
  }
  const Point2 relative = subtract(sample, first);
  const double along = (relative.x * direction.x + relative.y * direction.y) / segment_length;
  const double cross_track = std::abs(cross(relative, direction)) / segment_length;
  return along >= -0.5 * detection_length - kEpsilon &&
         along <= segment_length + 0.5 * detection_length + kEpsilon &&
         cross_track <= 0.5 * detection_width + kEpsilon;
}

}  // namespace

bool approximatelyEqual(double first, double second, double tolerance)
{
  return std::abs(first - second) <= tolerance;
}

double polygonArea(const Polygon & polygon)
{
  if (polygon.size() < 3) {
    return 0.0;
  }
  double twice_area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto & first = polygon[index];
    const auto & second = polygon[(index + 1) % polygon.size()];
    twice_area += first.x * second.y - first.y * second.x;
  }
  return 0.5 * twice_area;
}

RegionResult makeRectangle(const Point2 & lower_left, const Point2 & upper_right)
{
  if (upper_right.x <= lower_left.x || upper_right.y <= lower_left.y) {
    throw std::invalid_argument("Rectangle requires upper-right above and right of lower-left.");
  }
  return {{
    lower_left,
    {upper_right.x, lower_left.y},
    upper_right,
    {lower_left.x, upper_right.y}}, 0.0};
}

RegionResult makeIsoscelesTrapezoid(
  const Point2 & lower_left, const Point2 & upper_right,
  const Point2 & lower_right)
{
  if (lower_right.x <= lower_left.x) {
    throw std::invalid_argument("Trapezoid right-bottom must be right of left-bottom.");
  }
  const double bottom_y = 0.5 * (lower_left.y + lower_right.y);
  if (upper_right.y <= bottom_y) {
    throw std::invalid_argument("Trapezoid upper-right must be above the corrected bottom edge.");
  }
  const double center_x = 0.5 * (lower_left.x + lower_right.x);
  if (upper_right.x <= center_x) {
    throw std::invalid_argument("Trapezoid upper-right must be right of its symmetry axis.");
  }
  const Point2 corrected_lower_left{lower_left.x, bottom_y};
  const Point2 corrected_lower_right{lower_right.x, bottom_y};
  const Point2 upper_left{2.0 * center_x - upper_right.x, upper_right.y};
  Polygon polygon{corrected_lower_left, corrected_lower_right, upper_right, upper_left};
  if (polygonArea(polygon) <= kEpsilon) {
    throw std::invalid_argument("Trapezoid area is zero or vertex order is invalid.");
  }
  return {polygon, std::abs(lower_left.y - lower_right.y)};
}

Polygon insetConvexPolygon(const Polygon & polygon, double margin)
{
  if (polygon.size() < 3 || polygonArea(polygon) <= kEpsilon) {
    throw std::invalid_argument("Inset requires a counter-clockwise convex polygon.");
  }
  if (margin < 0.0) {
    throw std::invalid_argument("Safety margin cannot be negative.");
  }
  if (approximatelyEqual(margin, 0.0)) {
    return polygon;
  }

  std::vector<Point2> origins;
  std::vector<Point2> directions;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const Point2 direction = subtract(polygon[(index + 1) % polygon.size()], polygon[index]);
    const double length = std::hypot(direction.x, direction.y);
    if (length < kEpsilon) {
      throw std::invalid_argument("Polygon contains a zero-length edge.");
    }
    const Point2 inward_normal{-direction.y / length, direction.x / length};
    origins.push_back(add(polygon[index], scale(inward_normal, margin)));
    directions.push_back(direction);
  }

  Polygon inset;
  inset.reserve(polygon.size());
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const std::size_t previous = (index + polygon.size() - 1) % polygon.size();
    inset.push_back(intersectInfiniteLines(
      origins[previous], directions[previous], origins[index], directions[index]));
  }
  if (polygonArea(inset) <= kEpsilon) {
    throw std::invalid_argument("Safety margin removes the entire working region.");
  }
  return inset;
}

std::vector<Point2> generateFootprintAwareBoustrophedonPath(
  const Polygon & coverage_region, const Polygon & motion_region,
  double detection_width, double detection_length, double maximum_spacing,
  const std::string & sweep_direction, const std::string & start_corner)
{
  if (detection_width <= 0.0 || detection_length <= 0.0) {
    throw std::invalid_argument("Detection footprint dimensions must be positive.");
  }
  if (maximum_spacing <= 0.0) {
    throw std::invalid_argument("Track spacing must be positive.");
  }
  const bool horizontal = sweep_direction == "horizontal";
  if (!horizontal && sweep_direction != "vertical") {
    throw std::invalid_argument("Sweep direction must be horizontal or vertical.");
  }
  validateStartCorner(start_corner);
  const auto [minimum, maximum] = bounds(coverage_region, horizontal);
  const double span = maximum - minimum;
  if (span <= kEpsilon) {
    throw std::invalid_argument("Coverage region has no sweep span.");
  }
  const double usable_span = std::max(0.0, span - detection_width);
  const double interval_ratio = usable_span / maximum_spacing;
  const int line_count = std::max(
    1, static_cast<int>(std::ceil(interval_ratio - kLineCountTolerance)) + 1);
  const double spacing = line_count == 1 ? 0.0 : usable_span / static_cast<double>(line_count - 1);
  const bool start_low = start_corner.rfind("lower_", 0) == 0;
  const bool start_left = start_corner.compare(start_corner.size() - 4, 4, "left") == 0;
  const bool reverse_order = horizontal ? !start_low : !start_left;

  std::vector<Point2> path;
  path.reserve(static_cast<std::size_t>(2 * line_count));
  for (int line_index = 0; line_index < line_count; ++line_index) {
    const int ordered_index = reverse_order ? line_count - 1 - line_index : line_index;
    const double coordinate = span <= detection_width ?
      0.5 * (minimum + maximum) :
      minimum + 0.5 * detection_width + spacing * static_cast<double>(ordered_index);
    auto segment = clipScanLine(coverage_region, coordinate, horizontal);
    const Point2 extension = horizontal ?
      Point2{0.5 * detection_length, 0.0} : Point2{0.0, 0.5 * detection_length};
    segment.first = subtract(segment.first, extension);
    segment.second = add(segment.second, extension);
    if (!insideConvex(motion_region, segment.first) || !insideConvex(motion_region,
        segment.second))
    {
      throw std::invalid_argument(
              "Detection footprint requires a robot-centre waypoint outside motion_region.");
    }
    bool forward_along_line = horizontal ? start_left : start_low;
    if (line_index % 2 == 1) {
      forward_along_line = !forward_along_line;
    }
    if (!forward_along_line) {
      std::swap(segment.first, segment.second);
    }
    path.push_back(segment.first);
    path.push_back(segment.second);
  }
  return path;
}

std::vector<Point2> makeTopEdgeFinishingScan(
  const Polygon & coverage_region, const Polygon & motion_region,
  double detection_width, double detection_length, const Point2 & entry)
{
  if (detection_width <= 0.0 || detection_length <= 0.0) {
    throw std::invalid_argument("Detection footprint dimensions must be positive.");
  }
  if (coverage_region.size() < 3U) {
    throw std::invalid_argument("Coverage region requires at least three points.");
  }
  const auto [minimum_y, maximum_y] = bounds(coverage_region, true);
  // The footprint reaches half its width above the line, so this is the highest
  // line whose swept band still tops out exactly on the region edge. A region
  // shorter than the footprint is covered whole by one centred line.
  const double coordinate = maximum_y - minimum_y <= detection_width ?
    0.5 * (minimum_y + maximum_y) :
    maximum_y - 0.5 * detection_width;
  auto segment = clipScanLine(coverage_region, coordinate, true);
  const Point2 extension{0.5 * detection_length, 0.0};
  segment.first = subtract(segment.first, extension);
  segment.second = add(segment.second, extension);
  if (!insideConvex(motion_region, segment.first) ||
    !insideConvex(motion_region, segment.second))
  {
    // PROJECT_GUIDE 10.7 requires the finishing line and its transition to lie
    // in motion_region. Refusing here leaves the caller a task it can execute,
    // rather than one the executor will reject at goal acceptance.
    return {};
  }
  // Enter from the end nearer the last scan so the transition onto the line is
  // the short one, not a traverse back across the whole region.
  const double to_first = std::hypot(segment.first.x - entry.x, segment.first.y - entry.y);
  const double to_second = std::hypot(segment.second.x - entry.x, segment.second.y - entry.y);
  if (to_second < to_first) {
    std::swap(segment.first, segment.second);
  }
  return {segment.first, segment.second};
}

double sampledCoverageRatio(
  const Polygon & coverage_region, const std::vector<Point2> & scan_path,
  double detection_width, double detection_length, int samples_per_axis)
{
  if (scan_path.size() % 2 != 0 || detection_width <= 0.0 || detection_length <= 0.0 ||
    samples_per_axis <= 0)
  {
    throw std::invalid_argument("Coverage evaluation received invalid input.");
  }
  const auto [minimum_x, maximum_x] = bounds(coverage_region, false);
  const auto [minimum_y, maximum_y] = bounds(coverage_region, true);
  std::size_t inside_count = 0U;
  std::size_t covered_count = 0U;
  for (int row = 0; row < samples_per_axis; ++row) {
    for (int column = 0; column < samples_per_axis; ++column) {
      const Point2 sample{
        minimum_x + (maximum_x - minimum_x) * (static_cast<double>(column) + 0.5) /
        static_cast<double>(samples_per_axis),
        minimum_y + (maximum_y - minimum_y) * (static_cast<double>(row) + 0.5) /
        static_cast<double>(samples_per_axis)};
      if (!insideConvex(coverage_region, sample)) {
        continue;
      }
      ++inside_count;
      for (std::size_t index = 0; index < scan_path.size(); index += 2U) {
        if (coveredByFootprint(
            sample, scan_path[index], scan_path[index + 1U], detection_width, detection_length))
        {
          ++covered_count;
          break;
        }
      }
    }
  }
  return inside_count == 0U ? 0.0 :
         static_cast<double>(covered_count) / static_cast<double>(inside_count);
}

}  // namespace climbot_coverage
