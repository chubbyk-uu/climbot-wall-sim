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

#include "climbot_gazebo/pressure_sensor.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace climbot_gazebo::vacuum
{
PressureSensor::PressureSensor(SensorParameters p)
: p_(p)
{
  for (double x : {p.rate_hz, p.noise_pa, p.bias_pa, p.delay_s, p.minimum_pa, p.maximum_pa}) {
    if (!std::isfinite(x)) {throw std::invalid_argument("nonfinite pressure sensor parameter");}
  }
  if (p.rate_hz <= 0 || p.rate_hz > 1000 || p.noise_pa < 0 || p.delay_s < 0 ||
    p.delay_s > 10 || p.minimum_pa <= 0 || p.maximum_pa <= p.minimum_pa)
  {throw std::invalid_argument("invalid pressure sensor domain");}
  reset();
}
void PressureSensor::reset()
{
  random_.seed(p_.seed);gaussian_.reset();queue_.clear();sequence_ = 0;nextSlot_ = 0;
  lastTime_ = -1;held_ = 0;haveHeld_ = false;fault_ = 0;
}
void PressureSensor::fault(int mode)
{
  if (mode < 0 || mode > 2) {throw std::invalid_argument("invalid pressure sensor fault");}
  if (mode != fault_) {queue_.clear();fault_ = mode;}
}
std::optional<PressureSample> PressureSensor::update(double time, double pressure)
{
  if (!std::isfinite(time) || time < 0 || !std::isfinite(pressure) || time < lastTime_)
  {throw std::invalid_argument("invalid sensor simulation time/input");}
  if (time == lastTime_) {return std::nullopt;}
  // Gazebo's first unpaused PreUpdate is at dt, not at zero. Never fabricate
  // an acquisition at the reset snapshot; begin on the next absolute grid tick.
  if (lastTime_ < 0) {nextSlot_ = static_cast<uint64_t>(std::ceil(time * p_.rate_hz - 1e-10));}
  lastTime_ = time;
  if (time + 1e-10 >= static_cast<double>(nextSlot_) / p_.rate_hz) {
    // Enforce the configured grid instead of silently skipping acquisition slots.
    if (std::abs(time - static_cast<double>(nextSlot_) / p_.rate_hz) > 1e-8)
    {throw std::invalid_argument("sensor rate must align with physical step");}
    double raw = pressure + p_.bias_pa + p_.noise_pa * gaussian_(random_);
    double value = std::clamp(raw, p_.minimum_pa, p_.maximum_pa);
    if (fault_ != 2 || !haveHeld_) {held_ = value;haveHeld_ = true;}
    if (fault_ == 2) {value = held_;}
    PressureSample sample{time, time + p_.delay_s, value,
      fault_ == 2 ? 0.0 : p_.noise_pa * p_.noise_pa, sequence_++,
      fault_ != 2 && raw != value};
    ++nextSlot_;
    if (fault_ != 1) {queue_.push_back(sample);}
  }
  if (!queue_.empty() && time + 1e-10 >= queue_.front().delivery_s) {
    auto sample = queue_.front();queue_.pop_front();sample.delivery_s = time;return sample;
  }
  return std::nullopt;
}
}  // namespace climbot_gazebo::vacuum
