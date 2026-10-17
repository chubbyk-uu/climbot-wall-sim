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

#ifndef CLIMBOT_GAZEBO__PRESSURE_SENSOR_HPP_
#define CLIMBOT_GAZEBO__PRESSURE_SENSOR_HPP_
#include <cstdint>
#include <deque>
#include <optional>
#include <random>

namespace climbot_gazebo::vacuum
{
struct SensorParameters
{
  double rate_hz{100}, noise_pa{10}, bias_pa{25}, delay_s{.02};
  double minimum_pa{20000}, maximum_pa{120000};
  uint32_t seed{4817};
};
struct PressureSample
{
  double stamp_s{}, delivery_s{}, pressure_pa{}, variance_pa2{};
  uint64_t sequence{};
  bool saturated{};
};
// Faults: 0 normal, 1 dropout, 2 stuck value with a fresh acquisition stamp.
// No physics input is mutated. Random draws advance at every scheduled sample.
class PressureSensor
{
public:
  explicit PressureSensor(SensorParameters parameters);
  void reset();
  void fault(int mode);
  std::optional<PressureSample> update(double time_s, double pressure_pa);
  int fault() const {return fault_;}
  uint64_t samples() const {return sequence_;}

private:
  SensorParameters p_;
  std::mt19937 random_;
  std::normal_distribution<double> gaussian_{0, 1};
  std::deque<PressureSample> queue_;
  uint64_t sequence_{};
  uint64_t nextSlot_{};
  double lastTime_{-1}, held_{};
  bool haveHeld_{};
  int fault_{};
};
}  // namespace climbot_gazebo::vacuum
#endif
