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

#include "climbot_gazebo/clock_throttle.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace climbot_gazebo
{

namespace
{

double rateHz(uint64_t count, int64_t first_ns, int64_t last_ns)
{
  if (count < 2U || last_ns <= first_ns) {
    return 0.0;
  }
  return 1.0e9 * static_cast<double>(count - 1U) / static_cast<double>(last_ns - first_ns);
}

}  // namespace

ClockThrottle::ClockThrottle(double publish_rate_hz)
: requested_rate_hz_(publish_rate_hz)
{
  if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be positive and finite");
  }
  const double period = 1.0e9 / publish_rate_hz;
  if (period < 1.0 || period > static_cast<double>(std::numeric_limits<int64_t>::max())) {
    throw std::invalid_argument("publish_rate_hz produces an unsupported period");
  }
  period_ns_ = static_cast<int64_t>(std::llround(period));
}

void ClockThrottle::restartMeasurement(int64_t simulation_time_ns)
{
  inputs_ = 0;
  outputs_ = 0;
  first_input_ns_ = simulation_time_ns;
  last_measured_input_ns_ = simulation_time_ns;
  first_output_ns_ = simulation_time_ns;
  last_measured_output_ns_ = simulation_time_ns;
}

bool ClockThrottle::shouldPublish(int64_t simulation_time_ns)
{
  // A first sample and a simulator reset are the same case: nothing measured
  // before the jump describes the stream after it, so the window starts over
  // rather than reporting a rate averaged across a discontinuity.
  const bool restarted = !last_input_ns_ || simulation_time_ns < *last_input_ns_;
  if (restarted) {
    restartMeasurement(simulation_time_ns);
  }
  last_input_ns_ = simulation_time_ns;
  ++inputs_;
  last_measured_input_ns_ = simulation_time_ns;

  if (!restarted && simulation_time_ns < *next_publish_ns_) {
    return false;
  }
  // Schedule from the sample actually emitted. If the simulator jumps forward,
  // publish one current value rather than a burst of obsolete intermediate clocks.
  next_publish_ns_ = simulation_time_ns + period_ns_;
  ++outputs_;
  last_measured_output_ns_ = simulation_time_ns;
  return true;
}

int64_t ClockThrottle::periodNs() const
{
  return period_ns_;
}

double ClockThrottle::requestedRateHz() const
{
  return requested_rate_hz_;
}

double ClockThrottle::measuredInputRateHz() const
{
  return rateHz(inputs_, first_input_ns_, last_measured_input_ns_);
}

double ClockThrottle::measuredOutputRateHz() const
{
  return rateHz(outputs_, first_output_ns_, last_measured_output_ns_);
}

uint64_t ClockThrottle::inputs() const
{
  return inputs_;
}

uint64_t ClockThrottle::outputs() const
{
  return outputs_;
}

}  // namespace climbot_gazebo
