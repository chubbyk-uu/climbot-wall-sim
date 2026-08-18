#include <cmath>

#include <gtest/gtest.h>

#include "climbot_control/segment_duration.hpp"
#include "climbot_control/travel_profile.hpp"

namespace
{
/// The rated operating point the time-parameterised controller plans from:
/// symmetric ramps, both below the rate limiter that has to deliver them.
constexpr double kCruise = 0.20;
constexpr double kRated = 0.20;

/// What the distance-based controller actually does, and therefore what
/// estimateTravelDuration has to keep predicting: it accelerates at the rate
/// limiter's bound and brakes on a much gentler distance-to-stop curve.
constexpr double kBraking = 0.12;

double integrateSpeed(const climbot_control::TravelProfile & profile, int steps)
{
  // Trapezoidal integration of the sampled speed. The curve is piecewise
  // linear, so this converges on the analytic area rather than approaching it.
  const double step = profile.duration / steps;
  double total = 0.0;
  double previous = climbot_control::sampleTravel(profile, 0.0).speed;
  for (int index = 1; index <= steps; ++index) {
    const double speed = climbot_control::sampleTravel(profile, index * step).speed;
    total += 0.5 * (previous + speed) * step;
    previous = speed;
  }
  return total;
}
}  // namespace

TEST(TravelProfile, long_segments_reach_the_cruise_speed_and_coast)
{
  const auto profile = climbot_control::planTravel(4.31, kCruise, kRated, kRated);
  EXPECT_TRUE(profile.isTrapezoidal());
  EXPECT_DOUBLE_EQ(profile.peak_speed, kCruise);
  EXPECT_DOUBLE_EQ(profile.acceleration_duration, kCruise / kRated);
  EXPECT_DOUBLE_EQ(profile.braking_duration, kCruise / kRated);
  // The project document's trapezoidal case: T = D/vm + vm/av.
  EXPECT_NEAR(profile.duration, 4.31 / kCruise + kCruise / kRated, 1e-12);
}

TEST(TravelProfile, short_segments_brake_straight_out_of_the_ramp)
{
  // Below vm^2/av = 0.20 m the curve cannot reach the cruise speed.
  const auto profile = climbot_control::planTravel(0.10, kCruise, kRated, kRated);
  EXPECT_FALSE(profile.isTrapezoidal());
  EXPECT_DOUBLE_EQ(profile.coast_duration, 0.0);
  // The document's triangular case: vm' = sqrt(av * D), T = 2 * sqrt(D / av).
  EXPECT_NEAR(profile.peak_speed, std::sqrt(kRated * 0.10), 1e-12);
  EXPECT_NEAR(profile.duration, 2.0 * std::sqrt(0.10 / kRated), 1e-12);
}

TEST(TravelProfile, the_boundary_distance_belongs_to_the_trapezoid)
{
  const double boundary = kCruise * kCruise / kRated;
  const auto profile = climbot_control::planTravel(boundary, kCruise, kRated, kRated);
  EXPECT_DOUBLE_EQ(profile.peak_speed, kCruise);
  EXPECT_NEAR(profile.coast_duration, 0.0, 1e-12);
  const auto shorter = climbot_control::planTravel(
    boundary * 0.999, kCruise, kRated, kRated);
  EXPECT_LT(shorter.peak_speed, kCruise);
}

TEST(TravelProfile, the_curve_ends_stopped_at_the_far_end)
{
  for (const double distance : {0.05, 0.20, 0.40, 4.31}) {
    const auto profile = climbot_control::planTravel(distance, kCruise, kRated, kBraking);
    const auto finish = climbot_control::sampleTravel(profile, profile.duration);
    EXPECT_NEAR(finish.distance, distance, 1e-12) << "distance " << distance;
    EXPECT_DOUBLE_EQ(finish.speed, 0.0) << "distance " << distance;
    const auto after = climbot_control::sampleTravel(profile, profile.duration + 5.0);
    EXPECT_NEAR(after.distance, distance, 1e-12) << "distance " << distance;
    EXPECT_DOUBLE_EQ(after.speed, 0.0) << "distance " << distance;
  }
}

TEST(TravelProfile, the_sampled_speed_integrates_to_the_sampled_distance)
{
  for (const double distance : {0.05, 0.40, 4.31}) {
    const auto profile = climbot_control::planTravel(distance, kCruise, kRated, kBraking);
    EXPECT_NEAR(integrateSpeed(profile, 4000), distance, 1e-6) << "distance " << distance;
  }
}

TEST(TravelProfile, the_sampled_distance_never_goes_backwards)
{
  const auto profile = climbot_control::planTravel(4.31, kCruise, kRated, kBraking);
  double previous = -1.0;
  for (int index = 0; index <= 2000; ++index) {
    const double sampled = climbot_control::sampleTravel(
      profile, profile.duration * index / 2000.0).distance;
    EXPECT_GE(sampled, previous);
    previous = sampled;
  }
}

TEST(TravelProfile, before_the_start_it_reads_as_standstill_at_the_origin)
{
  const auto profile = climbot_control::planTravel(4.31, kCruise, kRated, kBraking);
  const auto before = climbot_control::sampleTravel(profile, -1.0);
  EXPECT_DOUBLE_EQ(before.distance, 0.0);
  EXPECT_DOUBLE_EQ(before.speed, 0.0);
}

// estimateTravelDuration is what the progress bar weights segments by. It now
// delegates to planTravel, so the two must agree exactly or the progress
// baseline moves under a change that was only meant to remove a duplicate
// formula.
TEST(TravelProfile, asymmetric_ramps_agree_with_the_duration_estimate)
{
  climbot_control::DurationModel model;
  model.cruise_speed = kCruise;
  model.linear_acceleration = kRated;
  model.braking_deceleration = kBraking;
  for (const double distance : {0.05, 0.20, 0.40, 1.21, 4.31}) {
    const auto profile = climbot_control::planTravel(distance, kCruise, kRated, kBraking);
    EXPECT_NEAR(
      profile.duration, climbot_control::estimateTravelDuration(distance, model), 1e-12)
      << "distance " << distance;
  }
}

TEST(TravelProfile, a_segment_with_nothing_left_to_travel_is_a_standstill)
{
  for (const double distance : {0.0, -0.5}) {
    const auto profile = climbot_control::planTravel(distance, kCruise, kRated, kBraking);
    EXPECT_DOUBLE_EQ(profile.duration, 0.0);
    EXPECT_DOUBLE_EQ(profile.distance, 0.0);
    const auto sample = climbot_control::sampleTravel(profile, 1.0);
    EXPECT_DOUBLE_EQ(sample.distance, 0.0);
    EXPECT_DOUBLE_EQ(sample.speed, 0.0);
  }
}

TEST(TravelProfile, non_positive_limits_are_a_configuration_error)
{
  EXPECT_THROW(climbot_control::planTravel(1.0, 0.0, kRated, kRated), std::invalid_argument);
  EXPECT_THROW(climbot_control::planTravel(1.0, kCruise, 0.0, kRated), std::invalid_argument);
  EXPECT_THROW(climbot_control::planTravel(1.0, kCruise, kRated, 0.0), std::invalid_argument);
  EXPECT_THROW(
    climbot_control::planTravel(1.0, std::nan(""), kRated, kRated), std::invalid_argument);
  const auto profile = climbot_control::planTravel(1.0, kCruise, kRated, kRated);
  EXPECT_THROW(climbot_control::sampleTravel(profile, std::nan("")), std::invalid_argument);
}
