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

// Erodes a counter-clockwise convex polygon by the symmetric translation
// envelope {-translation, +translation}.  Every returned point can move by
// either translation direction and still remain in the original polygon.
// Unlike an isotropic inset, a vertical envelope does not unnecessarily take
// room away from the left and right boundaries.
Polygon insetConvexPolygonForSymmetricTranslation(
  const Polygon & polygon, const Point2 & translation);

// Returns the counter-clockwise convex overlap of two counter-clockwise convex
// polygons. Throws when they do not overlap in an area, because that means no
// executable route remains for the selected region.
Polygon intersectConvexPolygons(const Polygon & first, const Polygon & second);

// True when every candidate vertex lies inside (or on) the convex container.
// Coverage regions are convex rectangles or isosceles trapezoids, so checking
// their vertices is sufficient to prove that the entire requested region fits.
bool containsConvexPolygon(const Polygon & container, const Polygon & candidate);

// Generates base_link SCAN line pairs inside drive_region.  Detection geometry
// affects coverage evaluation, not the user-selected robot route boundary.
std::vector<Point2> generateFootprintAwareBoustrophedonPath(
  const Polygon & drive_region, const Polygon & motion_region,
  double detection_width, double maximum_spacing,
  const std::string & sweep_direction, const std::string & start_corner);

// Returns the base_link SCAN line pair of one horizontal finishing scan along
// the top of drive_region, entered from whichever end lies nearer to entry. Returns an
// empty vector when either end would fall outside motion_region.  Only vertical
// sweeps can need this: a horizontal sweep's topmost line already tops out on
// the region edge (PROJECT_GUIDE 10.7).
std::vector<Point2> makeTopEdgeFinishingScan(
  const Polygon & drive_region, const Polygon & motion_region,
  const Point2 & entry);

// Estimates the fraction of coverage_region swept by SCAN line pairs.  Each
// pair is interpreted as a straight segment with a rectangular footprint.
double sampledCoverageRatio(
  const Polygon & coverage_region, const std::vector<Point2> & scan_path,
  double detection_width, double detection_length, int samples_per_axis = 300,
  double detection_forward_offset = 0.0);

double polygonArea(const Polygon & polygon);

bool approximatelyEqual(double first, double second, double tolerance = 1e-9);

}  // namespace climbot_coverage

#endif  // CLIMBOT_COVERAGE__COVERAGE_GEOMETRY_HPP_
