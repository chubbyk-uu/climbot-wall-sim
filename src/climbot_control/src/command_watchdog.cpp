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

#include "climbot_control/command_watchdog.hpp"

#include <cmath>
#include <stdexcept>

namespace climbot_control
{

CommandWatchdog::CommandWatchdog(double timeout_s)
: timeout_s_(timeout_s)
{
  if (!std::isfinite(timeout_s_) || timeout_s_ <= 0.0) {
    throw std::invalid_argument("Watchdog timeout must be positive and finite.");
  }
}

bool CommandWatchdog::accept(const Command & command, double received_time_s)
{
  if (!std::isfinite(command.linear) || !std::isfinite(command.angular) ||
    !std::isfinite(received_time_s))
  {
    command_ = {};
    have_command_ = false;
    return false;
  }
  command_ = command;
  received_time_s_ = received_time_s;
  have_command_ = true;
  return true;
}

bool CommandWatchdog::timedOut(double current_time_s) const
{
  return !std::isfinite(current_time_s) || !have_command_ ||
         current_time_s < received_time_s_ ||
         current_time_s - received_time_s_ > timeout_s_;
}

Command CommandWatchdog::commandAt(double current_time_s) const
{
  return timedOut(current_time_s) ? Command{} : command_;
}

}  // namespace climbot_control
