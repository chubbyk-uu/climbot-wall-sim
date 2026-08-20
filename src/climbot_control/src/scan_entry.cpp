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

#include "climbot_control/scan_entry.hpp"

#include <algorithm>
#include <cmath>

namespace climbot_control
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;
}  // namespace

LineOffset offsetFromLine(const Point2 & start, const Point2 & end, const Point2 & robot)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length = std::hypot(dx, dy);
  if (length <= 1e-9) {
    return {};
  }
  const double tx = dx / length;
  const double ty = dy / length;
  return {
    (robot.x - start.x) * tx + (robot.y - start.y) * ty,
    -(robot.x - start.x) * ty + (robot.y - start.y) * tx};
}

ScanEntry classifyScanOffset(double cross, double parallel, double maximum)
{
  const double offset = std::abs(cross);
  if (offset <= parallel) {
    return ScanEntry::LOCK_PARALLEL;
  }
  if (offset > maximum) {
    return ScanEntry::TOO_FAR;
  }
  return ScanEntry::ARC_ENTRY;
}

double firstScanEntryBudget(
  const Point2 & robot, const Point2 & first, const Point2 & second,
  const Point2 & lifted_target, const Point2 & gravity,
  double turn_slip_per_degree, double approach_tolerance)
{
  const double approach_heading = std::atan2(
    lifted_target.y - robot.y, lifted_target.x - robot.x);
  const double scan_heading = std::atan2(second.y - first.y, second.x - first.x);
  const double turn = std::abs(wrapAngle(scan_heading - approach_heading));
  const double drop = turn_slip_per_degree * turn * kDegreesPerRadian;
  const double reserved = std::hypot(
    lifted_target.x - first.x, lifted_target.y - first.y);
  const double residual = std::max(0.0, drop - reserved);
  const double gravity_norm = std::hypot(gravity.x, gravity.y);
  const double normal_share = gravity_norm <= 1e-9 ? 0.0 :
    std::abs(-std::sin(scan_heading) * gravity.x + std::cos(scan_heading) * gravity.y) /
    gravity_norm;
  return approach_tolerance + residual * normal_share;
}

bool turnSlipLooksStale(double turn_radians, double observed_drop, double slip_per_degree)
{
  const double degrees = std::abs(turn_radians) * kDegreesPerRadian;
  if (degrees < 10.0) {
    return false;
  }
  const double predicted = slip_per_degree * degrees;
  // Half the prediction, floored at 5 mm: a proportional band alone would make
  // small turns unfalsifiable and a fixed one would make large turns noisy.
  const double tolerance = std::max(0.005, 0.5 * predicted);
  return std::abs(observed_drop - predicted) > tolerance;
}

}  // namespace climbot_control
