#include <cmath>

#include <gtest/gtest.h>

#include "climbot_control/segment_duration.hpp"

namespace
{

/// The shipped control.yaml values, so the expectations below are comparable
/// with the recorded baselines rather than with an invented configuration.
climbot_control::DurationModel model()
{
  climbot_control::DurationModel value;
  value.cruise_speed = 0.20;
  value.linear_acceleration = 0.20;
  value.braking_deceleration = 0.12;
  value.max_turn_rate = 0.35;
  value.turn_acceleration = 1.00;
  value.settle_duration = 0.80;
  return value;
}

constexpr double kQuarterTurn = 1.5707963267948966;

}  // namespace

// Measured from results/coverage_trapezoid_horizontal_2026-08-14_trajectory.csv
// and the vertical rectangle baseline: every segment turns a quarter circle, so
// the pairs below isolate how length maps to duration.
TEST(SegmentDuration, matches_the_recorded_baselines_within_a_fifth)
{
  struct Case
  {
    double length;
    double measured;
  };
  const Case cases[] = {
    {3.89, 22.5}, {3.31, 22.3}, {2.74, 19.2},  // horizontal trapezoid scans
    {0.44, 9.4},                               // horizontal trapezoid transition
    {4.51, 28.0}, {0.40, 9.0},                 // vertical rectangle
  };
  for (const auto & entry : cases) {
    const double estimate =
      climbot_control::estimateSegmentDuration(entry.length, kQuarterTurn, model());
    EXPECT_LT(std::abs(estimate - entry.measured) / entry.measured, 0.20)
      << "length " << entry.length << " m estimated " << estimate << " s, measured "
      << entry.measured << " s";
  }
}

// This is the property the progress bar depends on. Weighting segments equally
// gave a transition the same share as a scan line, so the bar ran roughly 2.4
// times faster through transitions than through scans.
TEST(SegmentDuration, separates_a_short_transition_from_a_long_scan_line)
{
  const double transition =
    climbot_control::estimateSegmentDuration(0.44, kQuarterTurn, model());
  const double scan =
    climbot_control::estimateSegmentDuration(3.89, kQuarterTurn, model());
  EXPECT_GT(scan / transition, 2.0);
  EXPECT_LT(scan / transition, 3.0);
}

TEST(SegmentDuration, a_turn_still_costs_time_on_a_zero_length_segment)
{
  EXPECT_GT(climbot_control::estimateSegmentDuration(0.0, kQuarterTurn, model()), 1.0);
  EXPECT_EQ(climbot_control::estimateTravelDuration(0.0, model()), 0.0);
}

TEST(SegmentDuration, a_short_segment_never_claims_to_reach_cruise_speed)
{
  // Below the ramp distance the segment cannot reach cruise speed, so treating
  // it as cruise plus ramps would understate the time it takes.
  const double naive = 0.05 / model().cruise_speed;
  EXPECT_GT(climbot_control::estimateTravelDuration(0.05, model()), naive);
}

TEST(SegmentDuration, duration_grows_monotonically_with_length_and_angle)
{
  double previous = 0.0;
  for (double length = 0.0; length < 5.0; length += 0.25) {
    const double value = climbot_control::estimateTravelDuration(length, model());
    EXPECT_GE(value, previous);
    previous = value;
  }
  EXPECT_GT(
    climbot_control::estimateTurnDuration(kQuarterTurn, model()),
    climbot_control::estimateTurnDuration(0.1, model()));
}

TEST(SegmentDuration, non_finite_inputs_degrade_instead_of_poisoning_progress)
{
  const double nan = std::nan("");
  EXPECT_EQ(climbot_control::estimateTravelDuration(nan, model()), 0.0);
  EXPECT_TRUE(std::isfinite(climbot_control::estimateTurnDuration(nan, model())));
}
