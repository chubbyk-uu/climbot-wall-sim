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

#include <cstdint>
#include <optional>

namespace climbot_gazebo
{

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
};

}  // namespace climbot_gazebo

#endif  // CLIMBOT_GAZEBO__CLOCK_THROTTLE_HPP_
