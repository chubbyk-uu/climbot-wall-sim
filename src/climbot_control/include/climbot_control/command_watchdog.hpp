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

#ifndef CLIMBOT_CONTROL__COMMAND_WATCHDOG_HPP_
#define CLIMBOT_CONTROL__COMMAND_WATCHDOG_HPP_

#include "climbot_control/line_tracker.hpp"

namespace climbot_control
{

class CommandWatchdog
{
public:
  explicit CommandWatchdog(double timeout_s);

  bool accept(const Command & command, double received_time_s);
  Command commandAt(double current_time_s) const;
  bool timedOut(double current_time_s) const;

private:
  double timeout_s_;
  double received_time_s_{0.0};
  Command command_{};
  bool have_command_{false};
};

}  // namespace climbot_control

#endif  // CLIMBOT_CONTROL__COMMAND_WATCHDOG_HPP_
