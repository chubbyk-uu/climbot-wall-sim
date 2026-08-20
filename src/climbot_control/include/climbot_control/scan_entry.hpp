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

#ifndef CLIMBOT_CONTROL__SCAN_ENTRY_HPP_
#define CLIMBOT_CONTROL__SCAN_ENTRY_HPP_

#include "climbot_control/line_tracker.hpp"

namespace climbot_control
{

/// Where the robot sits relative to a line: how far along it, how far off it.
struct LineOffset
{
  double along{0.0};
  double cross{0.0};
};

/// Project a position onto a line, giving its along and signed cross offsets.
///
/// Returns zeros for a line with no length; the caller decides whether that is
/// a fault, because a zero-length nominal scan is one and a zero-length
/// reference the robot is standing on is not.
LineOffset offsetFromLine(const Point2 & start, const Point2 & end, const Point2 & robot);

/// What to do about the offset the robot ends a turn with.
enum class ScanEntry
{
  /// Small enough to accept: freeze a line parallel to the nominal one here.
  LOCK_PARALLEL,
  /// Too big to accept, small enough to drive out: one forward arc onto it.
  ARC_ENTRY,
  /// Beyond what the entry can recover. Nothing here can fix it.
  TOO_FAR,
};

/// Classify a post-turn cross offset. parallel <= maximum is the caller's
/// promise; the thresholds come from control.yaml and are validated there.
ScanEntry classifyScanOffset(double cross, double parallel, double maximum);

/// Worst normal offset entering the first scan line could leave, in metres.
///
/// The robot drives at the first waypoint, turns onto the scan heading, and
/// slides down the wall by an amount proportional to that turn. Some of that
/// slide is already paid for: the approach aims at a target lifted above the
/// waypoint, and the lift is the reservation. What is left over only shows up
/// as a scan offset to the extent the scan line runs across gravity - a scan
/// straight up or down the wall absorbs the slide along its own length, where
/// it is not an offset at all.
double firstScanEntryBudget(
  const Point2 & robot, const Point2 & first, const Point2 & second,
  const Point2 & lifted_target, const Point2 & gravity,
  double turn_slip_per_degree, double approach_tolerance);

/// Whether a measured turn drop disagrees with what the constant predicts.
///
/// The turn reservation trusts turn_slip_per_degree_m completely, so it is
/// only safe while that constant still describes this wall. Turns under ten
/// degrees are never judged: the drop is too small to tell from noise, and
/// reporting on them would train an operator to ignore the warning.
bool turnSlipLooksStale(
  double turn_radians, double observed_drop, double slip_per_degree);

}  // namespace climbot_control

#endif  // CLIMBOT_CONTROL__SCAN_ENTRY_HPP_
