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

#ifndef CLIMBOT_GAZEBO__CLOCK_THROTTLE_HPP_
#define CLIMBOT_GAZEBO__CLOCK_THROTTLE_HPP_

#include <array>
#include <cstdint>
#include <optional>

namespace climbot_gazebo
{

/// Fixed-space wall-clock gap statistics for one side of the throttle.
///
/// The throttle sits on the critical path of simulation time: if it is not
/// scheduled, /clock stops for every node. Measuring only the rate it delivers
/// cannot show that -- a stall and a clean stream average out the same. These
/// are kept for the input and the output separately, because equal gaps on
/// both sides mean the stall was upstream, while an output gap without a
/// matching input gap means the throttle itself was the blockage.
class GapStatistics
{
public:
  static constexpr std::array<double, 4> kThresholdsS{0.050, 0.100, 0.200, 0.250};

  void add(double gap_s);
  uint64_t samples() const {return samples_;}
  double maxS() const {return max_s_;}
  uint64_t atLeast(std::size_t index) const {return at_least_[index];}

private:
  uint64_t samples_{};
  double max_s_{};
  std::array<uint64_t, kThresholdsS.size()> at_least_{};
};

/// Select the newest input clock at no more than one configured simulation-time rate.
///
/// Only rates the input can be divided into are reachable. With a 1 ms Gazebo
/// step the ladder is 1000, 500, 333.3, 250, 200 Hz and so on; a request that
/// falls between two rungs cannot be honoured and lands on the lower one. That
/// is a property of the input, not a defect here, so the measurement below
/// exists to report what was actually delivered rather than let a log repeat
/// back the number that was asked for.
class ClockThrottle
{
public:
  explicit ClockThrottle(double publish_rate_hz);

  bool shouldPublish(int64_t simulation_time_ns);
  int64_t periodNs() const;
  double requestedRateHz() const;

  /// Measured over the window since the stream last restarted. Zero until two
  /// samples of that kind have been seen.
  double measuredInputRateHz() const;
  double measuredOutputRateHz() const;
  uint64_t inputs() const;
  uint64_t outputs() const;

  /// Wall-clock spacing of arrivals and of publications. Fed by the node,
  /// which owns the steady clock; this class stays free of ROS and of time.
  void recordInputGap(double gap_s) {input_gaps_.add(gap_s);}
  void recordOutputGap(double gap_s) {output_gaps_.add(gap_s);}
  const GapStatistics & inputGaps() const {return input_gaps_;}
  const GapStatistics & outputGaps() const {return output_gaps_;}

private:
  void restartMeasurement(int64_t simulation_time_ns);

  double requested_rate_hz_{};
  int64_t period_ns_{};
  std::optional<int64_t> last_input_ns_;
  std::optional<int64_t> next_publish_ns_;

  uint64_t inputs_{};
  uint64_t outputs_{};
  int64_t first_input_ns_{};
  int64_t last_measured_input_ns_{};
  int64_t first_output_ns_{};
  int64_t last_measured_output_ns_{};

  GapStatistics input_gaps_;
  GapStatistics output_gaps_;
};

}  // namespace climbot_gazebo

#endif  // CLIMBOT_GAZEBO__CLOCK_THROTTLE_HPP_
