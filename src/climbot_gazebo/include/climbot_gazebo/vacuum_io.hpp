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

#ifndef CLIMBOT_GAZEBO__VACUUM_IO_HPP_
#define CLIMBOT_GAZEBO__VACUUM_IO_HPP_
#include "climbot_gazebo/pressure_sensor.hpp"
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <array>
#include <memory>
#include <mutex>
#include <vector>

namespace climbot_gazebo::vacuum
{
class VacuumIO
{
public:
  explicit VacuumIO(const std::shared_ptr<sdf::Element> & sdf);
  void reset();
  std::vector<double> apply(double time);
  double openingArea() const {return openingArea_;}
  void publish(double time, std::vector<double> & state);

private:
  struct Command {double time{};int kind{}, index{}, value{};};
  void enqueue(int kind, int index, int value);
  void execute(const Command & command);
  std::unique_ptr<PressureSensor> sensor_;
  gz::transport::Node::Publisher pressure_, ambient_, truth_;
  std::mutex mutex_;
  std::vector<Command> pending_, events_;
  std::size_t nextEvent_{};
  std::array<bool, 3> enabled_{true, true, true}, failed_{};
  uint64_t revision_{}, rejected_{};
  double nextTruth_{}, truthPeriod_{};
  std::optional<PressureSample> last_;
  std::vector<std::pair<double, double>> openings_;
  double openingArea_{};
  // Destroy the transport subscriptions before the callback-owned state.
  gz::transport::Node node_;
};
}  // namespace climbot_gazebo::vacuum
#endif
